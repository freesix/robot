#pragma once 
#include "serial_imu_interface/msg/raw_imu.hpp"
#include "serial_imu_interface/msg/rpy_imu.hpp"
#include <pthread.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#define  OA_NMEA_STR_LEN 32

#define  IMU_DATA_CMD_END_TAG			(unsigned char*)("\r\n")
#define  IMU_DATA_CMD_JZ_TAG			(unsigned char*)("JZ")
#define  IMU_DATA_CMD_RAWIMU_TAG		(unsigned char*)("$RAWIMU")
#define  IMU_DATA_CMD_IMURPY_TAG		"$IMURPY"

#define  CSL_IMU_CFG_CMD_BACK			"OK"

#define  SUPPORT_IMU_UART_RCV_BUFF_MAX_LEN			5*1024


#pragma pack(1)
typedef struct RAWIMU_DATA_HEX_T
{
    char  Header1;
    char  Header2;
    unsigned short Lenght;
    unsigned short MessId;
    unsigned int TimeTicks;

    float Gyro_x;
    float Gyro_y;
    float Gyro_z;
    float Accel_x;
    float Accel_y;
    float Accel_z;
    float Magn_x;
    float Magn_y;
    float Magn_z;
    char ImuTemp;
    unsigned int Crc32;
}RAWIMU_DATA_HEX_T;

typedef struct IMURPY_DATA_HEX_T
{
    char  Header1;
    char  Header2;
    unsigned short Lenght;
    unsigned short MessId;
    unsigned int TimeTicks;
    float Pitch;
    float Roll;
    float Yaw;
    float Q1;
    float Q2;
    float Q3;
    float Q4;
    char ImuTemp;
    unsigned int Crc32;
}IMURPY_DATA_HEX_T;
#pragma pack()

extern serial_imu_interface::msg::RawImu imudata;
extern serial_imu_interface::msg::RpyImu rpydata;
extern rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr Imu_Publisher;
extern pthread_mutex_t mutex;
extern pthread_cond_t cond;
extern int turn;
int imu_data_parse_thread_init(void);


