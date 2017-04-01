/*
 ============================================================================
 Name        : spydude.c
 Author      : 
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C, Ansi-style
 ============================================================================
 */

/*
 * git push -u origin master
 *
 *  Programa para trasmitir codigo a un datalogger y actualizarlo usando un bootloader
 *  Dado que el enlace va a ser serial wireless, en la parte de comunicaciones vamos a
 *  implementar un protocolo resistente a delays y  perdida de caracteres utilizando la
 *  trasmision de bloques y retrasmision completa.
 *  https://en.wikipedia.org/wiki/Intel_HEX
 *
 *  Parte 1:
 *  Leemos un archivo pasado en linea de comandos linea a linea y lo imprimimos en pantalla.
 *  https://www.gnu.org/software/libc/manual/html_node/Parsing-Program-Arguments.html
 *
 *  Parte 2:
 *  Armar c/bloque con la informacion de una pagina e imprimirla
 *  Cada linea tiene un delimitador de entrada y luego tiene el largo util de la linea.
 *  Como las lineas pueden ser de diferente largo, debemos usar este dato para parsearla
 *
 *  !! La memoria del AVR es en WORDS por lo tanto si la pagina es de tamanio 128, en realidad
 *  van 256 bytes.
 *  Por otro lado, las direcciones son la mitad de las que aparecen en el archivo ya que en cada
 *  posicion guardo 2 bytes.
 *
 *  Parte 3:
 *  Implementar la salida por puerto serial
 *  Debe ser no canonica e implementar sincronizacion entre threads.
 *
 *  Parte 4:
 *  Implementar el protocolo de trasmision por bloques con retrasmisiones.
 *
 *
 *  El protocolo implementa que el datalogger mande el tamanio de la pagina.
 *  En el protocolo debo controlar cuando los checksum coinciden con caracteres de control ya que
 *  esto desacomoda todo.
 *
 *
 */

#include "spydude.h"

const char *argp_program_version =
  "spydude 1.0";
const char *argp_program_bug_address =
  "<software@spymovil.com>";

/* Program documentation. */
static char doc[] =
  "spydude -- programa para enviar codigo a spyloggers";

/* A description of the arguments we accept. */
static char args_doc[] = "";

/* The options we understand. */
static struct argp_option options[] = {
  {"baud_rate", 'b', "BAUDRATE" , 0,  "Baud rate" },
  {"program", 'p', "FILE", 0, "program file to bootload" },
  {"serial_port", 's',"SERIAL_PORT", 0, "serial port" },
  {"serial verbose", 'v',"SERIAL_VERBOSE", 0, "serial verbose{0,1}" },
  { 0 }
};

/* Parse a single option. */
static error_t parse_opt (int key, char *arg, struct argp_state *state)
{
  /* Get the input argument from argp_parse, which we
     know is a pointer to our arguments structure. */
  struct arguments *arguments = state->input;

  switch (key)
    {
    case 'b':
     arguments->baud_rate = arg;
     break;
    case 'p':
      arguments->program_file = arg;
      break;
    case 's':
      arguments->serial_port = arg;
      break;
    case 'v':
      arguments->serial_verbose = arg;
      break;
    }
  return 0;
}

/* Our argp parser. */
static struct argp argp = { options, parse_opt, args_doc, doc };

//------------------------------------------------------------------
 int main (int argc, char **argv)
{
  /* Default values. */
  arguments.program_file = "-";
  arguments.serial_port = "-";
  arguments.baud_rate = "-";
  arguments.serial_verbose = 0;

  PAGESIZE_WORDS = 128;
  PAGESIZE_BYTES = 2 *  PAGESIZE_WORDS;

  /* Parse our arguments; every option seen by parse_opt will
     be reflected in arguments. */
  argp_parse (&argp, argc, argv, 0, 0, &arguments);


  if ( !strcmp (arguments.program_file, "-")  ) {
	  printf("Debe proveer un nombre de archivo de codigo .hex\n");
	  exit (0);
  }

  if ( !strcmp (arguments.serial_port, "-")  ) {
	  printf("Debe proveer un puerto serial disponible\n");
	  exit (0);
  }

  if ( !strcmp( arguments.baud_rate, "-"  ) ) {
	  printf("Debe indicar la velocidad\n");
	  exit (0);
  }

  BAUDRATE = 0;
  if ( !strcmp( arguments.baud_rate, "9600"  ) ) {
	  BAUDRATE = B9600;
  } else   if ( !strcmp( arguments.baud_rate, "115200"  ) ) {
	  BAUDRATE = B115200;
  } else {
	  puts("Baud rate no aceptada");
	  exit(0);
  }


  printf ("PROGRAM_FILE = %s\n", arguments.program_file);
  printf ("SERIAL_PORT = %s\n", arguments.serial_port);
  printf ("BAUD_RATE = %d\n", arguments.baud_rate);


  open_serial_port();

  if (pthread_mutex_init(&mutex, NULL) != 0) {
    perror("pthread_mutex_init() error");
    exit(1);
  }

  if (pthread_mutex_init(&ring_buffer_mlock, NULL) != 0) {
	perror("pthread_mutex_init() error");
	exit(1);
  }

  if (pthread_cond_init(&condition, NULL) != 0) {
	perror("pthread_mutex_init() error");
	exit(1);
  }

  // Inicio el hilo de lectura (productor) del puerto serial
  if ( pthread_create ( &RX_listen_thread_handler, NULL, *RX_listen_thread, NULL) ) {
	  printf("ERROR::No puedo arrancar thread de RX_listen\n");
	  exit(1);
  }

  // Inicio el hilo de procesamiento ( consumer ) del puerto serial
  if ( pthread_create ( &RX_process_thread_handler, NULL, *RX_process_thread, NULL) ) {
	  printf("ERROR::No puedo arrancar thread de RX_listen\n");
	  exit(1);
  }


  while(1) {
	  sleep(10);
  }

  exit(0);
}
//------------------------------------------------------------------
