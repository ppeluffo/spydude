/*
 * spydude.h
 *
 *  Created on: 20 de mar. de 2017
 *      Author: pablo
 */

#ifndef SPYDUDE_H_
#define SPYDUDE_H_

#include <argp.h>
#include <stdbool.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <ctype.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <sys/select.h>

typedef unsigned char uint8;

/* Used by main to communicate with parse_opt. */
struct arguments
{
  char *program_file;
  char *serial_port;
  int baud_rate;
  int serial_verbose;
};

struct arguments arguments;

#define LINEBUFFERLENGHT 512

int load_one_page(int pagina);
void send_page(int pagina);
void leer_archivo(void);

pthread_mutex_t ring_buffer_mlock;
pthread_mutex_t mutex;
pthread_cond_t condition;

pthread_t RX_listen_thread_handler;
pthread_t RX_process_thread_handler;

void open_serial_port(void);
bool uart_putchar (const uint8 c);
bool  byte_send(const uint8 c);
void *RX_listen_thread(void *arg);
void *RX_process_thread(void *arg);

int PAGESIZE_WORDS;
int PAGESIZE_BYTES;

bool abort_signal;
bool link_established;
//------------------------------------------------------------------;

#endif /* SPYDUDE_H_ */
