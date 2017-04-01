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

#define LOCAL_BUFFER_LENGTH 32

struct local_buffer_struct {
	int ptr;
	uint8 data[LOCAL_BUFFER_LENGTH];
};


static bool push_into_ring_buffer(uint8 c);
static bool pop_from_ring_buffer(uint8 *c);
static void init_ring_buffer( void );
static void flush_ring_buffer( void );
static bool ring_buffer_is_empty( void );
static bool ring_buffer_is_full( void );

static bool push_into_local_buffer(uint8 c);
static void flush_local_buffer( void );

static struct ring_buffer_struct ring_buffer;
static struct local_buffer_struct local_buffer;

static int fd_serial;

//------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------
void *RX_process_thread(void *arg)
{
	// Thread que procesa los caracteres enviados, implementando el
	// protocolo


uint8 c;
int pagina = 0;
int page_size;
char *str;
int remote_checksum;
uint8 sent_checksum;

	puts("Processing serial...");

	// Fishing
	puts("Fishing...");
	while ( 1 ) {
		uart_putchar('S');
		sleep(1);
		while( ! ring_buffer_is_empty() ) {
			pop_from_ring_buffer(&c);
			push_into_local_buffer(c);
		}
		// Evaluo respuesta al SYNC
		if (strstr(local_buffer.data,"O\n"))
			break;
	}

	// Recibi 'O\r' (OK) .Dlg esta sincronizado.
	// Pido tamaño de pagina
	puts("DLG connected !!");
	flush_local_buffer();
	uart_putchar('B');
	puts("Page size ?");

	while (1)
	{

		// Duermo esperando recibir algun caracter.
		pthread_mutex_lock(&mutex); 			//mutex lock
		pthread_cond_wait(&condition, &mutex); 	//wait for the condition
		pthread_mutex_unlock(&mutex);

		while( ! ring_buffer_is_empty() ) {

			pop_from_ring_buffer(&c);
			push_into_local_buffer(c);

			// Muestro el caracter recibido
			if (isprint(c)) {
				printf("RX->[0x%02x]%c\n",c,c);
			} else {
				printf("RX->[0x%02x].\n",c);
			}

			// Analizo
			if ( c == '\n') {

				if ( (str = strchr(local_buffer.data,'W')) ) {
					// Block size
					str ++;
					page_size = (int)strtol(str, NULL, 16);
					printf("Tamano pagina recibido = %03d\n", page_size);
					PAGESIZE_BYTES = page_size;
					leerArchivo();
					puts("Reading file..");
					pagina = 1;
					flush_local_buffer();

				} else if ( strstr (local_buffer.data, "P\n")  ) {
					// Pide la pagina activa
					printf("Sending page %03d\n", pagina);
					if (load_one_page(pagina)) {
						sent_checksum = send_page(pagina);
					} else {
						puts("Empty page");
						uart_putchar('Z');		// No hay mas paginas
					}
					flush_local_buffer();

				} else if ( strstr (local_buffer.data, "M\n")  ) {
					// Avanzo una pagina
					pagina++;
					printf("Move forward to page %03d.\n", pagina);
					flush_local_buffer();

				} else if ( strstr (local_buffer.data, "A\n")  ) {
					// Abortar trasmision de pagina. ( Lo controlo al recibir los datos !!! )
					puts("Abort signal !!.\n");
					flush_local_buffer();

				} else if ( str = strchr (local_buffer.data, 'C')  ) {
					// Leo el checksum
					//remote_checksum = (int)strtol(str, NULL, 16);
					remote_checksum = local_buffer.data[1];
					if ( sent_checksum == remote_checksum ) {
						printf("Verificacion de checksum = (0x%02x). OK.\n", remote_checksum);
					} else {
						printf("Verificacion de checksum = (0x%02x). ERROR !!\n", remote_checksum);
					}
					flush_local_buffer();

				}	else if ( strstr (local_buffer.data, "X\n")  ) {
					// Exit
					puts("Exit..");
					exit(0);
				}

			}
		}
	}
}
//------------------------------------------------------------------
void *RX_listen_thread(void *arg)
{
 /*
  * Thread que atiende al puerto serial.
  * Leo en modo no canonico y las almaceno los caracteres en  un buffer circular para
  * que sean procesadas.
  * Oficia como productor de datos.
  * Cada dato que recibe lo guarda en un buffer circular y le avisa a la thread de proceso
  * que hay datos disponibles.
  *
  */

uint8 cChar;
int res;

	init_ring_buffer();

	puts("Listening...");

 	while (1) {

  		res = read(fd_serial, &cChar, 1);
		push_into_ring_buffer(cChar);

		if ( cChar == 'A') 			// Lo debo controlar aqui porque sino mientras trasmito la
			abort_signal = true;	// pagina no estoy controlando la recepcion de datos.

  		// Aviso al consumer que hay datos para trabajar.
 		pthread_cond_signal(&condition);

 		// Por amabilidad...
 		sched_yield();
 	}
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
  	cfsetispeed(&serialPortOptions, BAUDRATE);
  	cfsetospeed(&serialPortOptions, BAUDRATE);
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
  	//  serialPortOptions.c_iflag |= ICRNL;
  	serialPortOptions.c_iflag &= ~(IXON | IXOFF | IXANY);
  	serialPortOptions.c_iflag &= ~IGNCR;  // turn off ignore \r
  	serialPortOptions.c_iflag &= ~INLCR;  // turn off translate \n to \r
  	serialPortOptions.c_iflag &= ~ICRNL;  // turn off translate \r to \n
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
  	serialPortOptions.c_oflag &= ~ONLCR;  // turn off map \n  to \r\n
  	serialPortOptions.c_oflag &= ~OCRNL;  // turn off map \r to \n
  	serialPortOptions.c_oflag &= ~OPOST;  // turn off implementation defined output processing
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
static bool push_into_local_buffer(uint8 c)
{
	local_buffer.data[local_buffer.ptr] = c;
	local_buffer.ptr = ( local_buffer.ptr + 1 ) % LOCAL_BUFFER_LENGTH;
}
//------------------------------------------------------------------
static void flush_local_buffer( void )
{
	// Borra los punteros de rd y wr del buffer dejandolo en estado vacio.

	memset(local_buffer.data, '\0', LOCAL_BUFFER_LENGTH );
	local_buffer.ptr = 0;
}
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

