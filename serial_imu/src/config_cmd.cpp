// #include <serial/serial.h>
#include <stdio.h>
#include <semaphore.h>
#include <string.h>
#include <unistd.h>

#include "../include/serial_imu/config_cmd.hpp"
#include "../include/serial_imu/usart_driver.hpp"
extern sem_t cfg_bak_sem;
extern struct uart_port g_uartport;

static int sem_timedwait_millsecs(sem_t *sem, long long msecs);
static void uart_send_data(const char* Data, const unsigned int Len);

const char imu_cfg_cmd[50][50] = 
{
	"UNLOGALL\r\n",							//关闭IMU串口输出
	"LOG COM1 RAWIMUA ONTIME 0.01\r\n",		//以字符串形式输出IMU原始数据,100HZ
	"LOG COM1 RAWIMUB ONTIME 0.01\r\n",		//以二进制形式输出IMU原始数据,100HZ
	"LOG COM1 IMURPYA ONTIME 0.01\r\n",		//以字符串形式输出IMU姿态数据,100HZ
	"LOG COM1 IMURPYB ONTIME 0.01\r\n",		//以二进制形式输出IMU姿态数据,100HZ
	"SETAXISORIEN XYZ\r\n",					//IMU轴向，X朝右、Y超前、Z朝上
	"SERIALCONFIG COM1 460800\r\n",			//配置IMU串口1波特率为460800
	"SETRSTORT RPY\r\n"						//调平
};

static void uart_send_data(const char* Data, const unsigned int Len)
{

	if(Data!=NULL && Len>0)
	{
		if(g_uartport.fd>=0)
		{
			write(g_uartport.fd, Data, Len);
		}
	}
}

static int sem_timedwait_millsecs(sem_t *sem, long long msecs)
{
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	long long secs = msecs/1000;
	msecs = msecs%1000;
	
	long long add = 0;
	msecs = msecs*1000*1000 + ts.tv_nsec;
	add = msecs / (1000*1000*1000);
	ts.tv_sec += (add + secs);
	ts.tv_nsec = msecs%(1000*1000*1000);
	return sem_timedwait(sem, &ts);
}

int imu_cfg_cmd_send(const char *string, unsigned int slen)
{
	uart_send_data(string,slen);
	
	if(sem_timedwait_millsecs(&cfg_bak_sem,1000)!= 0)		//等待1S
	{
		return -1;
	}
	return 0;
}
