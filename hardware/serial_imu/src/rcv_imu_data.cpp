#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

#include "../include/serial_imu/usart_driver.hpp"
#include "../include/serial_imu/rcv_imu_data.hpp"

char *bRcvUartRingBuffer = NULL;
static unsigned int dwUartReadRingBufferPos = 0;
static unsigned int dwUartWriteRingBufferPos = 0;

struct uart_port g_uartport;
pthread_mutex_t	read_write_uart_mutex;

static void *rcv_imu_data_thread_loop(void *var);
/**
 * @brief 初始化串口
*/
static int usart_rcv_imu_data_init(void)
{
	struct tval *tvalp;
	
	g_uartport.fd = -1;
	if (open_uart_port(&g_uartport, (char *)DEFAULT_UART_DEV, atoi(DEFAULT_UART_BAUDRATE_STR)) < 0) // 串口打开失败
	{
		return -1;
	}
		

	if ((tvalp = (struct tval *)malloc(sizeof(*tvalp))) == NULL) // tvalp结构体未被初始化
	{
		close_uart_port(&g_uartport);
		
		return -1;
	}
	// 给tvalp串口描述结构体中个成员赋值
	tvalp->uart_portp = &g_uartport;
	tvalp->uart_dev =(char *) DEFAULT_UART_DEV;
	tvalp->uart_speed = atoi(DEFAULT_UART_BAUDRATE_STR);
}

/**
 * @brief 初始化数据接收线程
*/
int rcv_imu_data_thread_init(void)
{
	int err = 0;
	pthread_t tid;
	pthread_attr_t tattr;
	char cls_cmd[100] =  {0};
	usart_rcv_imu_data_init();
	err = pthread_mutex_init(&read_write_uart_mutex, NULL); // 线程初始化
	if(err==0)
	{
		printf("#######read_write_uart_mutex init ok#########\n");
	}
	
	bRcvUartRingBuffer=(char *)malloc(THE_UART_MAX_RING_BUFFER_LEN); // 分配buffer
	if(bRcvUartRingBuffer!=NULL)
	{
		memset(bRcvUartRingBuffer,0,THE_UART_MAX_RING_BUFFER_LEN);
	}
	else
	{
		printf("#######malloc bRcvUartRingBuffer address err#########\n");
		return -1;
	}

	err = pthread_attr_init(&tattr);
	if (err) 
	{
		printf("uart_rcv_thread_init: pthread_attr_init error: %s \n", strerror(err));
		return err;
	}
	err = pthread_attr_setdetachstate(&tattr, PTHREAD_CREATE_DETACHED);
	if (err == 0)
	{
		err = pthread_create(&tid, &tattr, rcv_imu_data_thread_loop, NULL);
	}
		
	pthread_attr_destroy(&tattr);
	
	sleep(1);          //阻塞一秒
	
	return err;
}

static void *rcv_imu_data_thread_loop(void *var)
{
	ssize_t n; 
	nfds_t nfd;
	int pollret;
	struct pollfd pfd[1];
	char tmpbuffer[1024];
	
	while(1)
	{
		pfd[0].fd = g_uartport.fd;
		pfd[0].events = POLLIN;
		nfd = 1;

		memset(tmpbuffer,0,sizeof(tmpbuffer));
		if ((pollret = poll(pfd, nfd, 500)) < 0)
		{	
			continue;
		}
		else if (pollret == 0)
		{
			continue;
		}
		if ((n = read(g_uartport.fd, tmpbuffer, sizeof(tmpbuffer))) < 0) 
		{
			continue;
		} 
		else if (n == 0)
		{
			continue;
		}
		#if 0
		int i=0;
        printf("<<");
		for(i = 0;i < n; i++)
		{
            printf("%02X ",tmpbuffer[i]);
		}
		printf(">>\r\n");
		#endif
		uart_wrtie_to_ring_buffer((unsigned char *)tmpbuffer,n);	
		usleep(1000);
	}

	pthread_detach(pthread_self());
	pthread_exit(NULL);
	
	return NULL;
}


int uart_wrtie_to_ring_buffer(unsigned char *data, int len)
{
	if(data==NULL|| len <=0)
	{
		return -1;
	}	
	pthread_mutex_lock(&read_write_uart_mutex);
	if(dwUartWriteRingBufferPos>=dwUartReadRingBufferPos)
	{
		if((dwUartWriteRingBufferPos+len)<THE_UART_MAX_RING_BUFFER_LEN)
		{
			memcpy(bRcvUartRingBuffer+dwUartWriteRingBufferPos,data,len);
			dwUartWriteRingBufferPos+=len;
		}
		else
		{
			unsigned int dwLaveDataLen=0;
			dwLaveDataLen=len-(THE_UART_MAX_RING_BUFFER_LEN-dwUartWriteRingBufferPos);
			memcpy(bRcvUartRingBuffer+dwUartWriteRingBufferPos,data,(THE_UART_MAX_RING_BUFFER_LEN-dwUartWriteRingBufferPos));
			if(dwLaveDataLen<=dwUartReadRingBufferPos)
			{
				memcpy(bRcvUartRingBuffer,data+(THE_UART_MAX_RING_BUFFER_LEN-dwUartWriteRingBufferPos),dwLaveDataLen);
				dwUartWriteRingBufferPos=dwLaveDataLen;
			}
			else
			{
				memcpy(bRcvUartRingBuffer,data+(THE_UART_MAX_RING_BUFFER_LEN-dwUartWriteRingBufferPos),dwUartReadRingBufferPos);
				dwUartWriteRingBufferPos=dwUartReadRingBufferPos;
			}
		}
	}
	else
	{
		if((unsigned int)len<=dwUartReadRingBufferPos-dwUartWriteRingBufferPos)
		{
			memcpy(bRcvUartRingBuffer+dwUartWriteRingBufferPos,data,len);
			dwUartWriteRingBufferPos+=len;
		}
		else
		{
			memcpy(bRcvUartRingBuffer+dwUartWriteRingBufferPos,data,dwUartReadRingBufferPos-dwUartWriteRingBufferPos);
			dwUartWriteRingBufferPos=dwUartReadRingBufferPos;
		}
	}
	pthread_mutex_unlock(&read_write_uart_mutex);
	return 0;
}

unsigned int uart_read_ring_buffer(unsigned char* Data, const unsigned int Len)
{
	unsigned int cnt = Len;
	unsigned int buflen = 0;
	unsigned int WritePlUart2BufPos = 0;
	if(cnt < 1)
	{
		return 0;
	}
	pthread_mutex_lock(&read_write_uart_mutex);
	WritePlUart2BufPos = dwUartWriteRingBufferPos;
	if(dwUartReadRingBufferPos!=WritePlUart2BufPos)
	{
		if(dwUartReadRingBufferPos<WritePlUart2BufPos)
		{
			buflen=WritePlUart2BufPos-dwUartReadRingBufferPos;
			if(buflen <= 0)
			{
				pthread_mutex_unlock(&read_write_uart_mutex);
				return 0;
			}
			if(cnt>buflen)
			{
				cnt=buflen;
			}
			memcpy(Data,(bRcvUartRingBuffer+dwUartReadRingBufferPos),cnt);
			dwUartReadRingBufferPos+=cnt;
			if(dwUartReadRingBufferPos>=THE_UART_MAX_RING_BUFFER_LEN)
				dwUartReadRingBufferPos=0;
		}
		else
		{
			buflen=THE_UART_MAX_RING_BUFFER_LEN-dwUartReadRingBufferPos+WritePlUart2BufPos;
			if(buflen >= THE_UART_MAX_RING_BUFFER_LEN)
			{
				pthread_mutex_unlock(&read_write_uart_mutex);
				return 0;
			}
			if(cnt>buflen)
			{
				cnt=buflen;
			}
			if(cnt<(THE_UART_MAX_RING_BUFFER_LEN-dwUartReadRingBufferPos))
			{
				memcpy(Data,bRcvUartRingBuffer+dwUartReadRingBufferPos,cnt);
				dwUartReadRingBufferPos+=cnt;
				if(dwUartReadRingBufferPos>=THE_UART_MAX_RING_BUFFER_LEN)
				{
					dwUartReadRingBufferPos=0;
				}	
			}
			else
			{
				memcpy(Data,bRcvUartRingBuffer+dwUartReadRingBufferPos,THE_UART_MAX_RING_BUFFER_LEN-dwUartReadRingBufferPos);
				buflen=cnt-(THE_UART_MAX_RING_BUFFER_LEN-dwUartReadRingBufferPos);
				memcpy(Data+(THE_UART_MAX_RING_BUFFER_LEN-dwUartReadRingBufferPos),bRcvUartRingBuffer,buflen);
				dwUartReadRingBufferPos=buflen;
			}
		}
		pthread_mutex_unlock(&read_write_uart_mutex);
		return cnt;
	}
	else
	{
		pthread_mutex_unlock(&read_write_uart_mutex);
		return 0;
	}
}