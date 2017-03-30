/*
 * spydude_serial.c
 *
 *  Created on: 22 de mar. de 2017
 *      Author: pablo
 */

/*------------------------------------------------------------------
 * Nombre		: initSerialPort
 * Descripcion	: Se encarga de abrir el puero serial que atiende al dlg e
 * 				: inicializarlo con los parametros que requiere.
 * 				: Devuelve 0 en caso de exito o 1 en error.
 *
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 * IMPORTANTE
 * Verificar como quedo configurado el puerto serial con el comando
 * > stty -a -F /dev/ttyXXX
 * La salida es algo del estilo:
 * speed 4800 baud; rows 0; columns 0; line = 0;
 * intr = <undef>; quit = <undef>; erase = <undef>; kill = <undef>; eof = ^D; eol = <undef>; eol2 = <undef>; swtch = <undef>; start = <undef>; stop = <undef>;
 * susp = <undef>; rprnt = <undef>; werase = <undef>; lnext = <undef>; flush = <undef>; min = 1; time = 5;
 * -parenb -parodd cs8 -hupcl cstopb cread clocal -crtscts
 * -ignbrk brkint -ignpar -parmrk -inpck -istrip -inlcr -igncr -icrnl -ixon -ixoff -iuclc -ixany -imaxbel -iutf8
 * -opost -olcuc -ocrnl -onlcr -onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0 ff0
 * -isig icanon -iexten -echo -echoe -echok -echonl -noflsh -xcase -tostop -echoprt -echoctl -echoke
 *
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
*/
#include "spydude.h"

#define RING_BUFFER_LENGTH 256

struct ring_buffer_struct {
	int push_ptr;
	int pop_ptr;
	int nro_recds;
	uint8 data[RING_BUFFER_LENGTH];
};

static bool push_into_ring_buffer(uint8 c);
static bool pop_from_ring_buffer(uint8 *c);
static void init_ring_buffer( void );
static void flush_ring_buffer( void );
static bool ring_buffer_is_empty( void );
static bool ring_buffer_is_full( void );

static struct ring_buffer_struct ring_buffer;

static int fd_serial;

//------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------
void *RX_process_thread(void *arg)
{
	// Thread que procesa los caracteres enviados, implementando el
	// protocolo
	// Como desde el datalogger vamos a enviar los datos recibidos en hexadecimal
	// con 2 nibbles, no usamos en el protocolo las letras A..F, a..f

uint8 c;
int pagina = 0;
uint8 lbuff[8];
int lptr = 0;
int page_size;

	puts("Processing serial...");

	// Fishing
	link_established = false;
	puts("Fishing...");
	while ( ! link_established ) {
		uart_putchar('S');
		sleep(1);
	}

	// Recivi 'O' (OK) .Dlg esta sincronizado.
	leerArchivo();
	puts("Reading file..");
	pagina = 0;

	while (1) {

		// Espero recibir algun caracter.
		pthread_mutex_lock(&mutex); 			//mutex lock
		pthread_cond_wait(&condition, &mutex); 	//wait for the condition
		pthread_mutex_unlock(&mutex);

		while( ! ring_buffer_is_empty() ) {

			pop_from_ring_buffer(&c);

			printf("RX->[0x%02x]%c\n",c,c);

			switch(c) {
			case 'O':					// OK.Dlg esta sincronizado. Me mando el tamaño de pagina.
				page_size = (int)strtol(lbuff, NULL, 16);
				printf("Tamano pagina recibido = %03d\n", page_size);
				PAGESIZE_BYTES = page_size;
				leerArchivo();
				puts("Reading file..");
				pagina = 0;
				break;
			case 'P':	// Page
				printf("Sending page %03d\n", pagina);
				if (load_one_page(pagina)) {
					send_page(pagina);
				} else {
					puts("Empty page");
					uart_putchar('Z');		// No hay mas paginas
				}
				break;
			case 'M':	// Move forward one page
				pagina++;
				printf("Move forward to page %03d.\n", pagina);
				break;
			case 'Q':	// Abort sending page
				abort_signal = true;
				puts("Abort signal received.");
				break;
			case 'X':	// Exit
				puts("Exit..");
				exit(0);
				break;
			default:	// Puede que sea un nro.
				lbuff[lptr] = c;
				lptr = ( lptr + 1) % 8;
				break;
			}
		}
	}
}
//------------------------------------------------------------------
void *RX_listen_thread(void *arg)
{
 /*
  * Thread que atiende al puerto serial.
  * Leo en modo no canonico y las almaceno los bytes en  un buffer circular para
  * que sean procesadas.
  * Oficia como productor de datos.
  *
  */

uint8 local_rx_buffer[RING_BUFFER_LENGTH];
int res;
int i;

	init_ring_buffer();

	puts("Listening...");

 	while (1) {

 		bzero(local_rx_buffer, sizeof(local_rx_buffer));
 		// Se bloquea con el read hasta que halla una linea completa
 		res = read(fd_serial, local_rx_buffer, RING_BUFFER_LENGTH);
 		// Lo guardo en el ring_buffer
  		// Elimino los \n y \r y paso los datos al buffer ppal.
 		for (i = 0; i < RING_BUFFER_LENGTH; i++)
 		{

 			if ( local_rx_buffer[i] == 'O')
 				link_established = true;

 			if ( local_rx_buffer[i] == '\0')
 			{
 				break;
 			}
 			else
 			{
 				push_into_ring_buffer(local_rx_buffer[i]);
 			//	printf("Rcvd 0x%02x\n",local_rx_buffer[i]);
 			}
 		}

 		// Aviso al consumer que hay datos para trabajar.
 		pthread_cond_signal(&condition);

 		// Por amabilidad...
 		sched_yield();
 	}
 }
//------------------------------------------------------------------
bool  byte_send(const uint8 c)
{
	// Envia c/u de los nibbles que forman el byte.

char data;
char txbuff[8];
int i;

	bzero(txbuff,sizeof(txbuff));
	sprintf(txbuff,"%02x.",c);
	i = 0;
	while ( ( data = txbuff[i++]) != '\0')
		write( fd_serial, &data, 1);

	return(true);

}
//------------------------------------------------------------------
bool uart_putchar (const uint8 c)
{

char data = c;

	// Trasmite un caracter por el puerto serial
	if ( write( fd_serial, &data, 1) < 0 ) {
		puts("ERROR::putchar_serial_port\n");
		return(false);
	}

	return (true);
}
//------------------------------------------------------------------
void open_serial_port(void)
{

  struct termios serialPortOptions;

	// Paso1: Abro el puerto serial para lectura canonica
	printf("Abriendo el puerto serial %s\n", arguments.serial_port);

	fd_serial = open(arguments.serial_port, O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY, 0777);
  	// O_RDWR: podemos leer y escribir
  	// O_NOCTTY: no queremos ser una terminal tty
  	// O_NODELAY: no le presto atencion al estado de la linea DCD

  	if (fd_serial < 0) {
  		printf("ERROR. Bye.\n");
  		exit (1);

  	} else {
  		fcntl(fd_serial, F_SETFL, 0);
  		// Seteo las flags del dispositivo
  		// Al ponerlas en 0 lo fijo en modo de bloqueo.
  		// Para que retorne enseguida debo poner FNDELAY
  	}

  	// Paso2: Configuro los parametros de acuerdo a lo requerido por el GPS
  	tcgetattr(fd_serial, &serialPortOptions); /* leo la configuracion actual del puerto */
  	/*
  	 * Control Flags
  	 * -------------
  	 * BAUDRATE: Fija la tasa bps. Podria tambien usar cfsetispeed y cfsetospeed.
  	 * CRTSCTS : control de flujo de salida por hardware (usado solo si el cable
  	 *           tiene todas las lineas necesarias Vea sect. 7 de Serial-HOWTO)
  	 * CS8     : 8n1 (8bit,no paridad,1 bit de parada)
  	 * CLOCAL  : conexion local, sin control de modem
  	 * CREAD   : activa recepcion de caracteres
  	*/
  	cfsetispeed(&serialPortOptions, B115200);
  	cfsetospeed(&serialPortOptions, B115200);
  	// gpsSerialPortOptions.c_cflag |= systemParameters.gpsBaudrate;
  	serialPortOptions.c_cflag &= ~CRTSCTS;
  	serialPortOptions.c_cflag |= CS8;
  	serialPortOptions.c_cflag |= CLOCAL;
  	serialPortOptions.c_cflag |= CREAD;

  	/*
  	 * Input flags
  	 * -----------
  	 * IGNPAR  : ignora los bytes con error de paridad
  	 * ICRNL   : mapea CR a NL (en otro caso una entrada CR del otro ordenador
  	 *  	      no terminaria la entrada) en otro caso hace un dispositivo en bruto
  	 *  	      (sin otro proceso de entrada)
  	 *  IXON | IXOFF | IXANY : Control de flujo. ( No lo necesito )
  	*/
  	serialPortOptions.c_iflag &= ~IGNPAR;
  	//	serialPortOptions.c_iflag &= ~ICRNL;
  	serialPortOptions.c_iflag |= ICRNL;
  	serialPortOptions.c_iflag &= ~(IXON | IXOFF | IXANY);

  	/*
  	 * Local flags
  	 * ------------
  	 * ICANON  : activa entrada canonica ( leo por lineas )
  	 *  	      desactiva todas las funcionalidades del eco, y no envia segnales al
  	 *  	      programa llamador
  	*/
  	serialPortOptions.c_lflag = 0;
  //	serialPortOptions.c_lflag |= ICANON;
  //	serialPortOptions.c_lflag &= ~ICANON;
  	serialPortOptions.c_lflag &= ~ECHO;
  	serialPortOptions.c_lflag &= ~ECHOE;
  //	gpsSerialPortOptions.c_lflag &= ~ISIG;

  	/*
  	 * Output flags
  	 * ------------
  	*/
  	serialPortOptions.c_oflag = 0;

  	/*
  	 * Caracteres de control
  	 * ---------------------
  	 * Inicializa todos los caracteres de control
  	 * los valores por defecto se pueden encontrar en /usr/include/termios.h,
  	 * y vienen dadas en los comentarios, pero no los necesitamos aqui
  	*/

  	serialPortOptions.c_cc[VINTR]    = 0;     /* Ctrl-c */
  	serialPortOptions.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
  	serialPortOptions.c_cc[VERASE]   = 0;     /* del */
  	serialPortOptions.c_cc[VKILL]    = 0;     /* @ */
  	serialPortOptions.c_cc[VEOF]     = 1;     /* Ctrl-d */
  	serialPortOptions.c_cc[VTIME]    = 0;     /* temporizador entre caracter, no usado */
  	serialPortOptions.c_cc[VMIN]     = 1;     /* bloqu.lectura hasta llegada de caracter. 1 */
  	serialPortOptions.c_cc[VSWTC]    = 0;     /* '\0' */
  	serialPortOptions.c_cc[VSTART]   = 0;     /* Ctrl-q */
  	serialPortOptions.c_cc[VSTOP]    = 0;     /* Ctrl-s */
  	serialPortOptions.c_cc[VSUSP]    = 0;     /* Ctrl-z */
  	serialPortOptions.c_cc[VEOL]     = 0;     /* '\0' */
  	serialPortOptions.c_cc[VREPRINT] = 0;     /* Ctrl-r */
  	serialPortOptions.c_cc[VDISCARD] = 0;     /* Ctrl-u */
  	serialPortOptions.c_cc[VWERASE]  = 0;     /* Ctrl-w */
  	serialPortOptions.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
  	serialPortOptions.c_cc[VEOL2]    = 0;     /* '\0' */

  	// 	ahora limpiamos la linea del modem
  	tcflush(fd_serial, TCIFLUSH);

  	//	y activamos la configuracion del puerto NOW
  	tcsetattr(fd_serial,TCSANOW,&serialPortOptions);

}
//------------------------------------------------------------------
// FUNCIONES BASICAS DE MANEJO DEL RING_BUFFER de USO PRIVADO
//------------------------------------------------------------------
static bool push_into_ring_buffer(uint8 c)
{
	// Si hay lugar almaceno un dato en el ring buffer.

int ret = false;

	if ( ! ring_buffer_is_full() ) {
		pthread_mutex_lock(&ring_buffer_mlock);
		ring_buffer.data[ring_buffer.push_ptr] = c;
		ring_buffer.nro_recds++;
		ring_buffer.push_ptr = ( ring_buffer.push_ptr + 1) % RING_BUFFER_LENGTH;
		ret = true;
		pthread_mutex_unlock(&ring_buffer_mlock);
	}
	return(ret);

}
//------------------------------------------------------------------
static bool pop_from_ring_buffer(uint8 *c)
{

	// Si hay lugar almaceno un dato en el ring buffer.

int ret = 0;

	if ( ! ring_buffer_is_empty() ) {
		pthread_mutex_lock(&ring_buffer_mlock);
		*c = ring_buffer.data[ring_buffer.pop_ptr];
		ring_buffer.nro_recds--;
		ring_buffer.pop_ptr = ( ring_buffer.pop_ptr + 1) % RING_BUFFER_LENGTH;
		ret = true;
		pthread_mutex_unlock(&ring_buffer_mlock);
	}
	return(ret);
}
//------------------------------------------------------------------
static void init_ring_buffer( void )
{
// Inicializa la estructura del puerto serial.

	pthread_mutex_lock(&ring_buffer_mlock);
	ring_buffer.nro_recds = 0;
	ring_buffer.pop_ptr = 0;
	ring_buffer.push_ptr = 0;
	memset( &ring_buffer.data, '\0', RING_BUFFER_LENGTH );
	pthread_mutex_unlock(&ring_buffer_mlock);

	puts("Init ring buffer");
}
//------------------------------------------------------------------
static void flush_ring_buffer( void )
{
	// Borra los punteros de rd y wr del buffer dejandolo en estado vacio.

	pthread_mutex_lock(&ring_buffer_mlock);
	ring_buffer.nro_recds = 0;
	ring_buffer.pop_ptr = 0;
	ring_buffer.push_ptr = 0;
	pthread_mutex_unlock(&ring_buffer_mlock);

}
//------------------------------------------------------------------
static bool ring_buffer_is_empty( void )
{
// Retorna TRUE si el buffer esta vacio o FALSE si aún tiene datos.
int ret = false;

	pthread_mutex_lock(&ring_buffer_mlock);
	if ( ring_buffer.nro_recds == 0 )
		ret = true;
	pthread_mutex_unlock(&ring_buffer_mlock);
	return(ret);

}
//------------------------------------------------------------------
static bool ring_buffer_is_full( void )
{
// Retorna TRUE si el buffer esta lleno o FALSE si tiene espacio libre

int ret = false;

	pthread_mutex_lock(&ring_buffer_mlock);
	if ( ring_buffer.nro_recds == RING_BUFFER_LENGTH )
		ret = true;
	pthread_mutex_unlock(&ring_buffer_mlock);
	return(ret);

}
//------------------------------------------------------------------

