#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <serial/serial.h>
#include <sensor_msgs/msg/imu.hpp>
#include <cstdint>

#define SWAP16(x) ((uint16_t)((((x) & 0x00FFU) << 8) | (((x) & 0xFF00U) >> 8)))
#define U8_TO_U16(high, low) ((((uint16_t)(high) << 8) | (low)))
#define U8_TO_I16(high, low) ((int16_t)((((uint16_t)(high) << 8) | (low))))
#define BUF_SIZE 24

typedef struct
{
    uint16_t head;
    int16_t accx;
    int16_t accy;
    int16_t accz;
    int16_t gyrx;
    int16_t gyry;
    int16_t gyrz;
    int16_t euly;
    int16_t eulp;
    int16_t eulr;
    int16_t time;
    int16_t crc;

}raw_data;

typedef struct
{
    float acc[3];
    float gyr[3];
    float quat[4];

}imu_data;


class imu_handle : public serial::Serial{
public:
    imu_handle(const std::string &port = "",
                  uint32_t baudrate = 9600,
                  serial::Timeout timeout = serial::Timeout(),
                  serial::bytesize_t bytesize = serial::eightbits,
                  serial::parity_t parity = serial::parity_none,
                  serial::stopbits_t stopbits = serial::stopbits_one,
                  serial::flowcontrol_t flowcontrol = serial::flowcontrol_none)  
         : serial::Serial(port, baudrate, timeout, bytesize, parity, stopbits, flowcontrol){}

    bool decode(imu_data &imudata, const uint8_t buffer[]);

private:
    void rawtoimu(imu_data &imudata, raw_data &rawdata);

    void rpytoquat(float r, float p, float y, float q[4]);

};
