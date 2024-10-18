#pragma once 
#include <poll.h>
#include <pthread.h>
#include <termios.h>



struct uart_port 
{
	char *device_lock_file;
	struct termios tiosfd; 		/* the saved UART port termios structure */
	struct termios tio; 		/* now opt UART port termios structure */
	int fd;						/* the UART port opend fd */
};

struct tval 
{
	struct uart_port *uart_portp;
	char *uart_dev;
	int uart_speed;
};

void close_uart_port(struct uart_port *uart_port);
int open_uart_port(struct uart_port *uart_port, char *dev, int baudrate);