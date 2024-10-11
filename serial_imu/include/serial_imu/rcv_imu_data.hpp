#pragma once 

#define DEFAULT_UART_DEV				"/dev/ttyUSB0"
#define DEFAULT_UART_BAUDRATE_STR		"460800"

#define THE_UART_MAX_RING_BUFFER_LEN	(1024*4)


int rcv_imu_data_thread_init(void);
int uart_wrtie_to_ring_buffer(unsigned char *data, int len);
unsigned int uart_read_ring_buffer(unsigned char* Data, const unsigned int Len);
