#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "../include/serial_imu/usart_driver.hpp"

static speed_t get_baudrate(int baudrate);
static int xtcsetattr(int fd, struct termios *tio, const char *device);
static void xtcgetattr(int fd, struct termios *t, struct termios *oldt);
static int _open_uart_port(struct uart_port *uart_port, char *dev, int baudrate, int plog);
/**
 * @brief 波特率数值转换为对应的speed_t类型
*/
static speed_t get_baudrate(int baudrate)
{
	speed_t speed;

	switch(baudrate) 
	{
		case 9600:
			{
				speed = B9600;
			} 
		break;
		case 115200:
			{
				speed = B115200;
			} 
		break;
		case 230400:
			{
				speed = B230400;
			} 
		break;
		case 460800:
			{
				speed = B460800;
			} 
		break;
		case 921600:
			{
				speed = B921600;
			} 
		break;
		default:
			{
				speed = B0;
			} 
		break;
	}
	return speed;
}

static void xtcgetattr(int fd, struct termios *t, struct termios *oldt)
{
    tcgetattr(fd, oldt);
	
    *t = *oldt;
    cfmakeraw(t);
}

static int xtcsetattr(int fd, struct termios *tio, const char *device)
{
    int ret = tcsetattr(fd, TCSAFLUSH, tio);

    if (ret) 
	{
        printf("can't tcsetattr for %s: %s ", device, strerror(errno));
    }
    return ret;
}

int open_uart_port(struct uart_port *uart_port, char *dev, int baudrate)
{
	return _open_uart_port(uart_port, dev, baudrate, 1);
}

/**
 * @brief 打开串口 
 * @param[in] uart_port 串口端口描述结构体 
 * @param[in] dev 端口号 
 * @param[in] baudrate 波特率 
 * @param[in] plog 是否输出日志
 * @return 串口开启标志位
*/
static int _open_uart_port(struct uart_port *uart_port, char *dev, int baudrate, int plog)
{
	speed_t speed;

	uart_port->device_lock_file = NULL;
	uart_port->fd = -1;

	speed = get_baudrate(baudrate);
	if (speed == B0) 
	{
		if (plog)
		{
			printf("invalid baudrate '%d' ", baudrate);
		}
		return -1;
	}
	// open device
	uart_port->fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (uart_port->fd < 0) 
	{
		if (plog)
		{
			printf("can't open '%s': %s ", dev, strerror(errno));
		}
		return -1;
	}
	fcntl(uart_port->fd, F_SETFL, O_RDWR);
	xtcgetattr(uart_port->fd, &(uart_port->tio), &(uart_port->tiosfd));
	// set device speed
	cfsetspeed(&(uart_port->tio), speed);
	if (xtcsetattr(uart_port->fd, &(uart_port->tio), dev)) 
	{
		if (uart_port->device_lock_file)
		{
			unlink(uart_port->device_lock_file);
		}	
		close(uart_port->fd);
		return -1;
	}
	return uart_port->fd;
}


void close_uart_port(struct uart_port *uart_port)
{
	if (uart_port->device_lock_file)
	{
		unlink(uart_port->device_lock_file);
		free(uart_port->device_lock_file);
		uart_port->device_lock_file = NULL;
	}

	if (uart_port->fd >= 0) 
	{
		tcsetattr(uart_port->fd, TCSAFLUSH, &(uart_port->tiosfd));
		close(uart_port->fd);
		uart_port->fd = -1;
	}
}