/*
 * spydude_aux.c
 *
 *  Created on: 20 de mar. de 2017
 *      Author: pablo
 */


#include "spydude.h"

struct line_struct {
	char line_str[LINEBUFFERLENGHT];
	char start_char;
	unsigned int length;
	uint8 address_H;
	uint8 address_L;
	uint8 eol;
	uint8 data[16];
	uint8 checksum;
};

struct page_struct {
	int number;
	int valid_data;
	unsigned int start_address;
	uint8 address_H;
	uint8 address_L;
	uint8 checksum;
	uint8 *buffer;
};

#define MEM_LENGTH_IN_BYTES	128*1024
uint8 program_hex_array[MEM_LENGTH_IN_BYTES];

static uint8 *pv_readLines(struct line_struct *rdline);
static void pv_parseLine(struct line_struct *rdline);
static void pv_parse_checksum(struct line_struct *rdline);
static void pv_parse_start(struct line_struct *rdline);
static void pv_parse_length(struct line_struct *rdline);
static void pv_parse_address(struct line_struct *rdline);
static void pv_parse_eol(struct line_struct *rdline);
static void pv_parse_data(struct line_struct *rdline);
static void hex2int(uint8 s[], uint8 d[], unsigned int largo);

static struct page_struct page;

static FILE *fd_inputFile;

//------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------
int load_one_page(int pagina)
{
	// Carga una pagina del array de memoria del programa
	// en una estructura 'page'.
	// En AVR cada posicion de memoria flash es WORD o sea 2 BYTEs.
	// Como cargo una pagina completa no tengo porque inicializar el
	// 'page'. Si lo hiciera deberia hacerlo con 0xFF que son los valores
	// por defecto de las posiciones no escritas.

int i;

	page.number = pagina;
	page.start_address = pagina * PAGESIZE_BYTES / 2;
	page.address_H = ( page.start_address >> 8 );
	page.address_L = ( page.start_address & 0x00FF );
	page.valid_data = false;

	page.checksum = 0;
	page.checksum += ( page.address_H ) & 0xFF;
	page.checksum += ( page.address_L ) & 0xFF;

	for ( i = 0; i < PAGESIZE_BYTES; i++ ) {
		page.buffer[i] = program_hex_array[pagina * PAGESIZE_BYTES + i];
		page.checksum += ( page.buffer[i] & 0xFF );
		if ( page.buffer[i] != 0xFF)
			page.valid_data = true;

	}

    //Now take 2's compliment
	page.checksum = 256 - page.checksum;

	return(page.valid_data);
}
//------------------------------------------------------------------
void send_page(int pagina )
{
	// Envia la pagina por el puerto serial y la despliega en pantalla
int i;
uint8 cks;
char out_buffer[LINEBUFFERLENGHT];

	abort_signal = false;

	// Caso 1: No quedan mas paginas
	if ( page.valid_data == false ) {
		bzero(out_buffer, sizeof(out_buffer));
		sprintf(out_buffer,"Empty page: %03d\r\n", pagina);
		printf("%s",out_buffer);
		return;
	}

	//  Caso 2: Hay una pagina activa ( con datos ).
	// 2.1: Trasmito la direccion y el checksum
	bzero(out_buffer, sizeof(out_buffer));

	if ( isprint(page.checksum) ) {
		  cks = page.checksum;
	} else {
		  cks = '.';
	}

	sprintf(out_buffer,"\nPage=%03d, address=0x%04x ( 0x%02x 0x%02x ), checksum=0x%02x[%c]\r\n",pagina, page.start_address, page.address_H, page.address_L, page.checksum, cks);
	printf("%s",out_buffer);

	uart_putchar('K');				// opCode: NORMAL: Inicio de pagina
	byte_send(page.address_H);
	byte_send(page.address_L);
	byte_send(page.checksum);

	// 2.2: Trasmito el payload de la pagina.
	for ( i = 0; i < PAGESIZE_BYTES; i++ ) {

		// Por si el dlg me pide abortar.
		if ( abort_signal ) {
			 abort_signal = false;
			return;
		}

		// Pagino la salida local
		if ( i%16 == 0 ) {
			printf("\r\n");
		}

		// Trasmito y muestro en pantalla
		bzero(out_buffer, sizeof(out_buffer));
		sprintf(out_buffer,"[%02x]", page.buffer[i]);
		printf("%s",out_buffer);
		if ( isprint(page.buffer[i]) ) {
			printf("%c ", page.buffer[i]);
		} else {
			printf(". ");
		}

		byte_send(page.buffer[i]);
	}

	uart_putchar('L');				// opCode: NORMAL: Fin de pagina
	printf("\r\n");

}
//------------------------------------------------------------------
void leerArchivo(void)
{
	// Leo todo el archivo y lo dejo en un array: program_hex_array[].
	// Lo leo de a lineas con readLines.

int i;
int mem_address;
struct line_struct rdline;

	// Inicializo el buffer de memoria para contener una pagina.
	page.buffer = (uint8 *)malloc( (PAGESIZE_BYTES + 1) * sizeof(uint8) );

	if (  (fd_inputFile = fopen(arguments.program_file, "r" ) )  ==  NULL ) {
		printf("ERROR: No es posible abrir el archivo %s\n", arguments.program_file);
		exit(1);
	}

	// Como los espacios vacios de memoria deben ser FF, lleno el buffer con estos.
	// Inicializo la mempage.
	for(i=0; i < MEM_LENGTH_IN_BYTES; i++)
		program_hex_array[i] = 255;

	// Leo el archivo y lo dejo en el array de memoria.
	while ( pv_readLines(&rdline) != NULL) {
		pv_parseLine(&rdline);

		// Linea de fin de archivo: salgo
		if ( rdline.eol == 1)
			break;

		// Salteo las lineas que no empiezan con :
		if ( rdline.start_char != ':')
			continue;

		// La direccion de memoria la debo dividir en 2 porque se almacenan de a WORDS
		mem_address = ( ( rdline.address_H << 8) + rdline.address_L );

		for(i=0; i < 16; i++) {
			program_hex_array[mem_address + i] = rdline.data[i];
		}

	}
}
//------------------------------------------------------------------
// FUNCIONES AUXILIARES BASICAS de USO PRIVADO
//------------------------------------------------------------------
static uint8 *pv_readLines(struct line_struct *rdline)
{

	// Leo lineas terminadas en \n del archivo con el programa
	// La deja almacenada en data_line.line_str.

uint8 *res;
int i;

	bzero(rdline->line_str, sizeof(rdline->line_str));
	res = fgets( rdline->line_str, LINEBUFFERLENGHT, fd_inputFile);
	if (  res != NULL ) {

		// Elimino caracteres de fin de linea
		for ( i = 0; i < strlen(rdline->line_str); i++) {
			if ( rdline->line_str[i] == '\n') rdline->line_str[i] = '\0';
			if ( rdline->line_str[i] == '\r') rdline->line_str[i] = '\0';
		}
	}

	return(res);

}
//------------------------------------------------------------------
static void pv_parseLine(struct line_struct *rdline)
{

	//printf("{%s}\n", data_line.line_str);

	pv_parse_start(rdline);
	pv_parse_length(rdline);
	pv_parse_address(rdline);
	pv_parse_eol(rdline);
	pv_parse_data(rdline);
	pv_parse_checksum(rdline);
}
//------------------------------------------------------------------
static void pv_parse_checksum(struct line_struct *rdline)
{
	// Byte que indica el checksum.
	// La posicion es variable porque depende del largo

	//: 10 4C8C 00 73657400726561640077726974650072 93
	//0 1  3    7  9

int bytes_to_get = 1;			// El largo son 1 bytes ( 2 nibbles + '\0' )
int nibbles_to_convert = 2;
int pos_in_rdline = 1 + 2 + 4 + 2 + rdline->length*2;
uint8 tmp_str[3];
uint8 hex_str[1];		// array para c/u de los bytes convertidos

	bzero(tmp_str, sizeof(tmp_str));
	// Copio los caracteres 1 y 2.( 2 nibbles )
	strncpy(tmp_str, &rdline->line_str[pos_in_rdline], nibbles_to_convert);
	hex2int(tmp_str, hex_str, bytes_to_get);
	rdline->checksum = hex_str[0];

}
//------------------------------------------------------------------
static void pv_parse_data(struct line_struct *rdline)
{
	// Linea de datos: hasta 16 bytes ( 32 nibbles )
	// El largo en bytes viene indicado por data_line.length.
	// Si es de menos de 16 bytes, la relleno con FF.
	// Determina el largo util ( payload ).
	//: 10 4C8C 00 73657400726561640077726974650072 93
	//0 1  3    7  9

int bytes_to_get = rdline->length;		// El largo son data_line.length bytes
int nibbles_to_convert = ( 2 * rdline->length );
int pos_in_rdline = 9;			// El largo arranca en la posicion 1
uint8 tmp_str[33];				// Como maximo puedo tener 32 nibbles + '\0'
uint8 hex_str[16];		// array para c/u de los bytes convertidos
int i;

	bzero(tmp_str, sizeof(tmp_str));
	for (i=0; i < 16; i++)
		hex_str[i] = 255;
	// Copio los caracteres 1 y 2.( 2 nibbles )
	strncpy(tmp_str, &rdline->line_str[pos_in_rdline], nibbles_to_convert);
	hex2int(tmp_str, hex_str, bytes_to_get);
	memcpy(rdline->data, hex_str, sizeof(hex_str));

}
//------------------------------------------------------------------
static void pv_parse_eol(struct line_struct *rdline)
{
	// Byte que indica la ultima linea
	//: 10 4C8C 00 7365740072656164007772697465007293
	//0 1  3    7

int bytes_to_get = 1;			// El eol son 1 bytes ( 2 nibbles + '\0' )
int nibbles_to_convert = 2;
int pos_in_rdline = 7;			// El largo arranca en la posicion 1
uint8 tmp_str[3];
uint8 hex_str[1];		// array para c/u de los bytes convertidos

	bzero(tmp_str, sizeof(tmp_str));
	// Copio los caracteres 7 y 8.( 2 nibbles )
	strncpy(tmp_str, &rdline->line_str[pos_in_rdline], nibbles_to_convert);
	hex2int(tmp_str, hex_str, bytes_to_get);
	rdline->eol = hex_str[0];

}
//------------------------------------------------------------------
static void pv_parse_address(struct line_struct *rdline)
{
	// Leemos la direccion de memoria de la linea ( 2 bytes )
	//: 10 4C8C 007365740072656164007772697465007293
	//0 1  3

int bytes_to_get = 2;			// El address son 2 bytes ( 4 nibbles + '\0' )
int nibbles_to_convert = 4;
int pos_in_rdline = 3;			// La direccion arranca en la posicion 3
uint8 tmp_str[5];
uint8 hex_str[2];		// array para c/u de los bytes convertidos

	bzero(tmp_str, sizeof(tmp_str));
	// Copio los caracteres 3,4,5,6.( 4 nibbles )
	strncpy(tmp_str, &rdline->line_str[pos_in_rdline], nibbles_to_convert);
	hex2int(tmp_str, hex_str, bytes_to_get);
	rdline->address_H = hex_str[0];
	rdline->address_L = hex_str[1];

}
//------------------------------------------------------------------
static void pv_parse_length(struct line_struct *rdline)
{
	// Determina el largo util ( payload ).
	//: 10 4C8C007365740072656164007772697465007293
	//0 1

int bytes_to_get = 1;			// El largo son 1 bytes ( 2 nibbles + '\0' )
int nibbles_to_convert = 2;
int pos_in_rdline = 1;			// El largo arranca en la posicion 1
uint8 tmp_str[3];
uint8 hex_str[1];		// array para c/u de los bytes convertidos

	bzero(tmp_str, sizeof(tmp_str));
	// Copio los caracteres 1 y 2.( 2 nibbles )
	strncpy(tmp_str, &rdline->line_str[pos_in_rdline], nibbles_to_convert);
	hex2int(tmp_str, hex_str, bytes_to_get);
	rdline->length = hex_str[0];

}
//------------------------------------------------------------------
static void pv_parse_start(struct line_struct *rdline)
{
	// Determina el caracter de START LINE
	//: 104C8C007365740072656164007772697465007293
	//0
	rdline->start_char = rdline->line_str[0];
}
//------------------------------------------------------------------
static void hex2int(uint8 s[], uint8 d[], unsigned int largo)
{
unsigned int res;
char a,b;
int i,j;
uint8 h1,h2;

	i = 0;
	j = 0;

	while(  (s[i] != '\0') &&  ( j <= largo ) ) {

		a = s[i];

		if(s[i] >='0' && s[i] <='9')
			h1= s[i] - '0';
		else if(s[i] >='a' && s[i] <='f')
			h1= s[i] - 'a' + 10;
		else if(s[i] >= 'A' && s[i] <='F')
			h1 = s[i] - 'A' + 10;

		i++;
		b = s[i];
		if(s[i] >='0' && s[i] <='9')
			h2 = s[i] - '0';
		else if(s[i] >='a' && s[i] <='f')
			h2 = s[i] -'a' + 10;
		else if(s[i] >='A' && s[i] <='F')
			h2 = s[i] -'A' + 10;

		i++;


		res = h1*16+h2;
		//printf ("%c%c {%d %d}<%d>\n",a,b,h1,h2,res);
		d[j] = res;
		j++;

	}

}
//------------------------------------------------------------------
