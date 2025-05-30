#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <semaphore.h>
#include "include/serial_imu/comm_interface.hpp"
#include "include/serial_imu/config_cmd.hpp"
#include "include/serial_imu/imu_data_parse.hpp"
#include "include/serial_imu/rcv_imu_data.hpp"
#include "include/serial_imu/usart_driver.hpp"
#include <sensor_msgs/msg/imu.hpp> 
#include <rclcpp/rclcpp.hpp>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>
#include <memory>
sem_t cfg_bak_sem;

using namespace std::chrono_literals;

/* private:
    void publish_imu_data() {
        auto imu_msg = sensor_msgs::msg::Imu();

        imu_msg.header.stamp = imudata.header.stamp;
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

        // 发布IMU消息
        imu_publisher_->publish(imu_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::TimerBase::SharedPtr time_;
}; */

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("imu_publisher_node");
    node->declare_parameter("imu_topic", "/imu/data");
    std::string Topic = node->get_parameter("imu_topic").as_string();
    Imu_Publisher = node->create_publisher<sensor_msgs::msg::Imu>(Topic, 10);

    sem_init(&cfg_bak_sem, 0, 0);
    rcv_imu_data_thread_init();
    imu_data_parse_thread_init();
    sleep(5);
    if(imu_cfg_cmd_send(imu_cfg_cmd[1],strlen(imu_cfg_cmd[1])) == 0)
	{
		printf("OK,LOG COM1 RAWIMUA ONTIME 0.01\r\n");
	}
	if(imu_cfg_cmd_send(imu_cfg_cmd[3],strlen(imu_cfg_cmd[3])) == 0)
	{
		printf("OK,LOG COM1 IMURPYA ONTIME 0.01\r\n");
	}
    
    rclcpp::spin(node);
	rclcpp::shutdown();
    return 0;
}

