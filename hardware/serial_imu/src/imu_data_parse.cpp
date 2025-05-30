#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <semaphore.h>
#include <poll.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/stat.h>
#include <sys/types.h>
// #include "serial_imu_interface/msg/raw_imu.hpp"
// #include "serial_imu_interface/msg/rpy_imu.hpp"
#include <iostream>


#include "../include/serial_imu/usart_driver.hpp"
#include "../include/serial_imu/rcv_imu_data.hpp"
#include "../include/serial_imu/config_cmd.hpp"
#include "../include/serial_imu/imu_data_parse.hpp"
#include "../include/serial_imu/comm_interface.hpp"

serial_imu_interface::msg::RawImu imudata;
serial_imu_interface::msg::RpyImu rpydata;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr Imu_Publisher;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
int turn = 1;
rclcpp::Clock time_(RCL_SYSTEM_TIME);
bool RAW_FLAG, PRY_FLAG;

extern sem_t cfg_bak_sem;

IMURPY_DATA_HEX_T rpydata_hex;
RAWIMU_DATA_HEX_T imudata_hex;

unsigned char bSendBuf[512] = {0};
unsigned char gImuParsBuf[SUPPORT_IMU_UART_RCV_BUFF_MAX_LEN];

static void *imu_data_parse_thread_loop(void *var);
DisposeFlag imu_data_parse(unsigned char* RData, const unsigned int RLen, 
	unsigned char** SAddr, unsigned int* OKLen);
void publish_imu_data(serial_imu_interface::msg::RawImu &imudata,
	serial_imu_interface::msg::RpyImu &rpydata);

int imu_data_parse_thread_init(void)
{
	int err;
	pthread_t tid;
	pthread_attr_t tattr;
	
	err = pthread_attr_init(&tattr);
	if (err) 
	{
		printf("pl_uart2_rcv_thread_init: pthread_attr_init error: %s \n", strerror(err));
		return err;
	}
	err = pthread_attr_setdetachstate(&tattr, PTHREAD_CREATE_DETACHED);
	if (err == 0)
	{
		err = pthread_create(&tid, &tattr, imu_data_parse_thread_loop, NULL);
	}
		
	pthread_attr_destroy(&tattr);
	
	sleep(1);          //阻塞一秒
	
	return err;

}


static void *imu_data_parse_thread_loop(void *var)
{
	unsigned int RLen = 0;
    static unsigned int dImuProDataRcvPos=0;
    static unsigned int bImuHasNoDataTimes=0;//No data time

	while(1)
	{   // 读取此次消息长度
		RLen = uart_read_ring_buffer((gImuParsBuf+dImuProDataRcvPos), (SUPPORT_IMU_UART_RCV_BUFF_MAX_LEN-dImuProDataRcvPos));
	
		if(RLen < 1) // 没有数据
		{
			bImuHasNoDataTimes++;
			if(bImuHasNoDataTimes >= 2000)
			{
				memset(gImuParsBuf,0,sizeof(gImuParsBuf));
				dImuProDataRcvPos=0;
				bImuHasNoDataTimes=0;
				printf("imu no data\r\n");
			}
			
			usleep(1000);	
			continue;
		}
		bImuHasNoDataTimes=0;
		dImuProDataRcvPos+=RLen;
		#if 0
		int i=0;
		printf("$$<<\r\n");
		for(i = 0; i < dImuProDataRcvPos ; i++)
			printf("%c", (unsigned char)gImuParsBuf[i]);
		printf(">>$$\r\n");
        dImuProDataRcvPos = 0;
		#endif
		
		DisposeFlag Df = NULL_RETURN;
		unsigned int OKDlen = 0;
		unsigned char* FindOKSaddr = NULL;
		unsigned int Ylen = 0;
		while(1)
		{
			if(dImuProDataRcvPos < 1)
				break;
			
			Df = imu_data_parse(gImuParsBuf, dImuProDataRcvPos, &FindOKSaddr, &OKDlen);
			
			if(Df == END_RETURN)
			{
				break;
			}
			else if(Df==CLEAR_BUF_RETURN)
			{	
				// RCLCPP_INFO(rclcpp::get_logger("imu"), "msg.ax: %f", imudata.accel_x);
				// RCLCPP_INFO(rclcpp::get_logger("imu"), "msg.ay: %f", imudata.accel_y);
				// RCLCPP_INFO(rclcpp::get_logger("imu"), "msg.az: %f", imudata.accel_z);
				publish_imu_data(imudata, rpydata);	
				Ylen = (unsigned int)(FindOKSaddr - gImuParsBuf);
				Ylen += OKDlen;
				Ylen = dImuProDataRcvPos - Ylen;
				if(Ylen > 0)
				{
					memmove(gImuParsBuf, (FindOKSaddr + OKDlen), Ylen);
				}
				else
				{
					Ylen=0;
				}
				memset((gImuParsBuf + Ylen), 0, SUPPORT_IMU_UART_RCV_BUFF_MAX_LEN-Ylen);
				dImuProDataRcvPos = Ylen;
			}
			else
			{
				Ylen = (unsigned int)(FindOKSaddr - gImuParsBuf);
				Ylen += OKDlen;
				Ylen = dImuProDataRcvPos - Ylen;
	
				if(Ylen > 0)
					memmove(FindOKSaddr, (FindOKSaddr + OKDlen), Ylen);
				else
					Ylen=0;
				memset((FindOKSaddr + Ylen), 0, SUPPORT_IMU_UART_RCV_BUFF_MAX_LEN - ((unsigned int)(FindOKSaddr - gImuParsBuf) + Ylen));
				dImuProDataRcvPos -= OKDlen;
			}
		}
		if(dImuProDataRcvPos>=SUPPORT_IMU_UART_RCV_BUFF_MAX_LEN)
        {
            dImuProDataRcvPos=0;
        }
        usleep(1000);
    }    
	return NULL;        
}
/**
 * @brief 解析imu数据
*/
DisposeFlag imu_data_parse(unsigned char* RData, const unsigned int RLen, unsigned char** SAddr, unsigned int* OKLen){
    unsigned int l = 0;
    unsigned char *t = 0;
	unsigned char* p = 0;
	unsigned int Crc32Val = 0;
    unsigned char* SAddr1 = NULL;
    DisposeFlag eReturn=END_RETURN;

    char bImuTmpBuf[OA_NMEA_STR_LEN]={0};
    // 如果数据为空,或者长度小于7
    if(RData == NULL || RLen < 7)
    {
        return END_RETURN;
    }
    if(FindDispose(RData, RLen, IMU_DATA_CMD_JZ_TAG, SAddr, OKLen) == MIDDLE_RETURN){  // 二进制格式
        SAddr1 = *SAddr;
        p = *SAddr + *OKLen;

        unsigned short datalen= 0;
		unsigned short datatype = 0;
		datalen = (p[1] << 8) | p[0];

        if(RLen - (SAddr1-RData) < datalen)
		{
			return END_RETURN;
		}
		datatype = (p[3]  <<  8) | p[2];

        #if 1
		switch(datatype)
		{
			case 0x0101:
				{
					memset((char*)&imudata_hex,0,sizeof(RAWIMU_DATA_HEX_T));
					memcpy((char*)&imudata_hex,(char*)SAddr1,sizeof(RAWIMU_DATA_HEX_T)-4);
					Crc32Val= Crc32Check((unsigned char*)&imudata_hex,sizeof(RAWIMU_DATA_HEX_T));

					memcpy((char*)&imudata_hex.Crc32,(char*)(p+datalen),4);

					if(Crc32Val == imudata_hex.Crc32)
					{
						unsigned char Tempbuf[200] = {0};
						memset(bSendBuf,0,sizeof(bSendBuf));
						memcpy(Tempbuf,(char*)&imudata_hex,sizeof(RAWIMU_DATA_HEX_T));
						for(int i = 0; i < sizeof(RAWIMU_DATA_HEX_T); i ++)
						{
							sprintf((char*)bSendBuf + i *3,"%02X ",Tempbuf[i]);
						}
						printf("%s\r\n",bSendBuf);
	
						*SAddr = SAddr1;
						*OKLen = datalen +6 ;
						return CLEAR_BUF_RETURN;
					}
					else
					{
						*SAddr = SAddr1;
						*OKLen = 2;
						return CLEAR_BUF_RETURN;
					}
				}
			break;
			case 0x0102:
				{
					memset((char*)&rpydata_hex,0,sizeof(IMURPY_DATA_HEX_T));
					memcpy((char*)&rpydata_hex,(char*)SAddr1,sizeof(IMURPY_DATA_HEX_T)-4);
					Crc32Val= Crc32Check((unsigned char*)&rpydata_hex,sizeof(IMURPY_DATA_HEX_T));

					memcpy((char*)&rpydata_hex.Crc32,(char*)(p+datalen),4);

					if(Crc32Val == rpydata_hex.Crc32)
					{
						unsigned char Tempbuf[200] = {0};
						memset(bSendBuf,0,sizeof(bSendBuf));
						memcpy(Tempbuf,(char*)&rpydata_hex,sizeof(IMURPY_DATA_HEX_T));
						for(int i = 0; i < sizeof(IMURPY_DATA_HEX_T); i ++)
						{
							sprintf((char*)bSendBuf + i *3,"%02X ",Tempbuf[i]);
						}
						printf("%s\r\n",bSendBuf);

							
						*SAddr = SAddr1;
						*OKLen = datalen +6 ;
						return CLEAR_BUF_RETURN;
					}
					else
					{
						*SAddr = SAddr1;
						*OKLen = 2;
						return CLEAR_BUF_RETURN;
					}
				}
			break;
			default:
				{
					return END_RETURN;
				}
			break;
	
		}
    }
    #endif
    else if(FindDispose(RData, RLen, IMU_DATA_CMD_RAWIMU_TAG,SAddr, OKLen) == MIDDLE_RETURN){
        SAddr1 = *SAddr;
        l = strlen((char*)IMU_DATA_CMD_END_TAG);
        t = FindString(*SAddr,RLen - (*SAddr - RData),IMU_DATA_CMD_END_TAG,l);
        if(t != 0){
            memset(bImuTmpBuf, 0 , sizeof(bImuTmpBuf));
            memcpy((char*)bImuTmpBuf, (char*)(t-8), 8);
            imudata.crc32 = strtoul(bImuTmpBuf, NULL, 16);
            Crc32Val = Crc32Check((unsigned char*)(SAddr1+1), (t-SAddr1)-10);
            if(Crc32Val == imudata.crc32){
                p = *SAddr + *OKLen +1;
				memset(&imudata,0,sizeof(imudata));

				memset(bImuTmpBuf,0,sizeof(bImuTmpBuf));
                FindString_token(&p, (unsigned int*)t,  ',',  bImuTmpBuf,  OA_NMEA_STR_LEN);
				imudata.timeticks = atoi(bImuTmpBuf);
	
				memset(bImuTmpBuf,0,sizeof(bImuTmpBuf));
				FindString_token(&p,  (unsigned int*)t,  ',',  bImuTmpBuf,  OA_NMEA_STR_LEN);
				imudata.gyro_x = atof(bImuTmpBuf);
				// RCLCPP_INFO(rclcpp::get_logger("imu"), "imu.gx: %s", bImuTmpBuf);
				// RCLCPP_INFO(rclcpp::get_logger("imu"), "msg.gx: %f", imudata.gyro_x);
				memset(bImuTmpBuf,0,sizeof(bImuTmpBuf));
				FindString_token(&p,  (unsigned int*)t,  ',',  bImuTmpBuf,  OA_NMEA_STR_LEN);
				imudata.gyro_y = atof(bImuTmpBuf);
				// RCLCPP_INFO(rclcpp::get_logger("imu"), "imu.gy: %s", bImuTmpBuf);
				// RCLCPP_INFO(rclcpp::get_logger("imu"), "msg.gy: %f", imudata.gyro_y);
				memset(bImuTmpBuf,0,sizeof(bImuTmpBuf));
				FindString_token(&p,  (unsigned int*)t,  ',',  bImuTmpBuf,  OA_NMEA_STR_LEN);
				imudata.gyro_z = atof(bImuTmpBuf);
				// RCLCPP_INFO(rclcpp::get_logger("imu"), "imu.gz: %s", bImuTmpBuf);
				// RCLCPP_INFO(rclcpp::get_logger("imu"), "msg.gz: %f", imudata.gyro_z);
				memset(bImuTmpBuf,0,sizeof(bImuTmpBuf));
				FindString_token(&p,  (unsigned int*)t,  ',',  bImuTmpBuf,  OA_NMEA_STR_LEN);
				imudata.accel_x = atof(bImuTmpBuf);
				// RCLCPP_INFO(rclcpp::get_logger("imu"), "imu.ax: %s", bImuTmpBuf);
				// RCLCPP_INFO(rclcpp::get_logger("imu"), "msg.ax: %f", imudata.accel_x);
				memset(bImuTmpBuf,0,sizeof(bImuTmpBuf));
				FindString_token(&p,  (unsigned int*)t,  ',',  bImuTmpBuf,  OA_NMEA_STR_LEN);
				imudata.accel_y = atof(bImuTmpBuf);
				// RCLCPP_INFO(rclcpp::get_logger("imu"), "imu.ay: %s", bImuTmpBuf);
				// RCLCPP_INFO(rclcpp::get_logger("imu"), "msg.ay: %f", imudata.accel_y);
				memset(bImuTmpBuf,0,sizeof(bImuTmpBuf));
				FindString_token(&p,  (unsigned int*)t,  ',',  bImuTmpBuf,  OA_NMEA_STR_LEN);
				imudata.accel_z = atof(bImuTmpBuf);
				// RCLCPP_INFO(rclcpp::get_logger("imu"), "imu.az: %s", bImuTmpBuf);
				// RCLCPP_INFO(rclcpp::get_logger("imu"), "msg.az: %f", imudata.accel_z);
				memset(bImuTmpBuf,0,sizeof(bImuTmpBuf));
				FindString_token(&p,  (unsigned int*)t,  ',',  bImuTmpBuf,  OA_NMEA_STR_LEN);
				imudata.magn_x = atof(bImuTmpBuf);

				memset(bImuTmpBuf,0,sizeof(bImuTmpBuf));
				FindString_token(&p,  (unsigned int*)t,  ',',  bImuTmpBuf,  OA_NMEA_STR_LEN);
				imudata.magn_y = atof(bImuTmpBuf);

				memset(bImuTmpBuf,0,sizeof(bImuTmpBuf));
				FindString_token(&p, (unsigned int*) t,  ',',  bImuTmpBuf,  OA_NMEA_STR_LEN);
				imudata.magn_z = atof(bImuTmpBuf);

				memset(bImuTmpBuf,0,sizeof(bImuTmpBuf));
				FindString_token(&p,  (unsigned int*)t,  ',',  bImuTmpBuf,  OA_NMEA_STR_LEN);
                int32_t sec = static_cast<int32_t>(atof(bImuTmpBuf));
                uint32_t nanosec = static_cast<uint32_t>((atof(bImuTmpBuf)-sec)*1e9);
                imudata.header.stamp.sec = sec;
                imudata.header.stamp.nanosec = nanosec;
				RAW_FLAG = true;
				#if 0
				memset(bSendBuf,0,sizeof(bSendBuf));
				sprintf((char*)bSendBuf,"$RAWIMU,%d,%0.5lf,%0.5lf,%0.5lf,%0.5lf,%0.5lf,%0.5lf,%0.5lf,%0.5lf,%0.5lf,%0.5lf*%X",
				    imudata.timeticks,imudata.gyro_x,imudata.gyro_y,imudata.gyro_z,imudata.accel_x,imudata.accel_y,imudata.accel_z,
					imudata.magn_x,imudata.magn_y,imudata.accel_z,Crc32Val);

				printf("%s\r\n",bSendBuf);
				#endif

				*SAddr = SAddr1;
				*OKLen = (t + l)  - *SAddr ;
				return CLEAR_BUF_RETURN;

            }
            else
			{
				*SAddr = SAddr1;
				*OKLen = 7;
				return CLEAR_BUF_RETURN;				
			}

        }
        else{
            return END_RETURN;
        }
    }
    else if(FindDispose(RData, RLen, (unsigned char*)IMU_DATA_CMD_IMURPY_TAG, SAddr, OKLen) == MIDDLE_RETURN)
    {
        SAddr1 = *SAddr;
        l = strlen((char*)IMU_DATA_CMD_END_TAG);
        t = FindString(*SAddr,RLen - (*SAddr - RData),IMU_DATA_CMD_END_TAG,l);
         if(t != 0)
        {
			memset(bImuTmpBuf,0,sizeof(bImuTmpBuf));
			memcpy((char*)bImuTmpBuf,(char*)(t-8),8);
			rpydata.crc32 = (unsigned int)strtol(bImuTmpBuf,NULL,16);
			Crc32Val = Crc32Check((SAddr1+1),(t - SAddr1)-10);
			if(Crc32Val == rpydata.crc32)
			{
				p = *SAddr + *OKLen +1;
				memset(&rpydata,0,sizeof(rpydata));

				memset(bImuTmpBuf,0,sizeof(bImuTmpBuf));
				FindString_token(&p,  (unsigned int*)t,  ',',  bImuTmpBuf,  OA_NMEA_STR_LEN);
				rpydata.timeticks = atoi(bImuTmpBuf);

				memset(bImuTmpBuf,0,sizeof(bImuTmpBuf));
				FindString_token(&p,  (unsigned int*)t,  ',',  bImuTmpBuf,  OA_NMEA_STR_LEN);
				rpydata.pitch= atof(bImuTmpBuf);

				memset(bImuTmpBuf,0,sizeof(bImuTmpBuf));
				FindString_token(&p,  (unsigned int*)t,  ',',  bImuTmpBuf,  OA_NMEA_STR_LEN);
				rpydata.roll = atof(bImuTmpBuf);

				memset(bImuTmpBuf,0,sizeof(bImuTmpBuf));
				FindString_token(&p,  (unsigned int*)t,  ',',  bImuTmpBuf,  OA_NMEA_STR_LEN);
				rpydata.yaw = atof(bImuTmpBuf);

				memset(bImuTmpBuf,0,sizeof(bImuTmpBuf));
				FindString_token(&p,  (unsigned int*)t,  ',',  bImuTmpBuf,  OA_NMEA_STR_LEN);
				rpydata.quaternion_w= atof(bImuTmpBuf);

				memset(bImuTmpBuf,0,sizeof(bImuTmpBuf));
				FindString_token(&p,  (unsigned int*)t,  ',',  bImuTmpBuf,  OA_NMEA_STR_LEN);
				rpydata.quaternion_x= atof(bImuTmpBuf);

				memset(bImuTmpBuf,0,sizeof(bImuTmpBuf));
				FindString_token(&p,  (unsigned int*)t,  ',',  bImuTmpBuf,  OA_NMEA_STR_LEN);
				rpydata.quaternion_y= atof(bImuTmpBuf);

				memset(bImuTmpBuf,0,sizeof(bImuTmpBuf));
				FindString_token(&p,  (unsigned int*)t,  ',',  bImuTmpBuf,  OA_NMEA_STR_LEN);
				rpydata.quaternion_z= atof(bImuTmpBuf);

				memset(bImuTmpBuf,0,sizeof(bImuTmpBuf));
				FindString_token(&p,  (unsigned int*)t,  ',',  bImuTmpBuf,  OA_NMEA_STR_LEN);
                int32_t sec = static_cast<int32_t>(atof(bImuTmpBuf));
                uint32_t nanosec = static_cast<uint32_t>((atof(bImuTmpBuf)-sec)*1e9);
                rpydata.header.stamp.sec = sec;
                rpydata.header.stamp.nanosec = nanosec;
				PRY_FLAG = true;

				#if 0
				memset(bSendBuf,0,sizeof(bSendBuf));
				sprintf((char*)bSendBuf,"$IMURPY,%d,%0.5lf,%0.5lf,%0.5lf,%0.5lf,%0.5lf,%0.5lf,%0.5lf,%0.5lf*%X",
					rpydata.timeticks,rpydata.pitch,rpydata.roll,rpydata.yaw,rpydata.quaternion_w,rpydata.quaternion_x,rpydata.quaternion_y,
					rpydata.quaternion_z,Crc32Val);

				printf("%s\r\n",bSendBuf);
				#endif

				*SAddr = SAddr1;
				*OKLen = (t + l)  - *SAddr ;
				return CLEAR_BUF_RETURN;
			}
            else
			{
				*SAddr = SAddr1;
				*OKLen = 7;
				return CLEAR_BUF_RETURN;
			}
        } 
        else
        {
            return END_RETURN;
        }
    }
    else if(FindDispose(RData, RLen, (unsigned char*)CSL_IMU_CFG_CMD_BACK,SAddr, OKLen) == MIDDLE_RETURN)
	{
		SAddr1 = *SAddr;
		l = strlen((char*)IMU_DATA_CMD_END_TAG);
		t = FindString(*SAddr,RLen - (*SAddr - RData),IMU_DATA_CMD_END_TAG,l);
		if(t != 0)
		{
			printf("IMU CLS BACK\r\n");
			sem_post(&cfg_bak_sem);
			*SAddr = SAddr1;
			*OKLen = (t + l)  - *SAddr ;
			return CLEAR_BUF_RETURN;
		}
		else
		{
			*SAddr = SAddr1;
			*OKLen = 2;
			return CLEAR_BUF_RETURN;
		}
	}
    else
    {
        return END_RETURN;
    }
    return eReturn;
}

void publish_imu_data(serial_imu_interface::msg::RawImu &imudata,
	serial_imu_interface::msg::RpyImu &rpydata) {
	
	auto imu_msg = sensor_msgs::msg::Imu();
	imu_msg.header.stamp = time_.now();
	imu_msg.header.frame_id = "imu_link";

	imu_msg.angular_velocity.x = imudata.gyro_x;
	imu_msg.angular_velocity.y = imudata.gyro_y;
	imu_msg.angular_velocity.z = imudata.gyro_z;

	imu_msg.linear_acceleration.x = imudata.accel_x;
	imu_msg.linear_acceleration.y = imudata.accel_y;
	imu_msg.linear_acceleration.z = imudata.accel_z;

	imu_msg.orientation.w = rpydata.quaternion_w;
	imu_msg.orientation.x = rpydata.quaternion_x;
	imu_msg.orientation.y = rpydata.quaternion_y;
	imu_msg.orientation.z = rpydata.quaternion_z;
	// RCLCPP_INFO(rclcpp::get_logger("imu"), "gx:%d,   gy:%d,   gz:%d", imu_msg.angular_velocity.x, 
		// imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);
	// RCLCPP_INFO(rclcpp::get_logger("imu"), "ax:%d,   ay:%d,   az:%d", imu_msg.linear_acceleration.x,
		// imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z); 
	// 发布IMU消息
	Imu_Publisher->publish(imu_msg);
}