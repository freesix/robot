#include <serial/serial.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "ch_serial.hpp"
#include <chrono>

#define GRA_ACC   9.8
#define DEG_TO_RAD  0.01745329
#define BUF_SIZE    1024

using namespace std::chrono_literals; // 使用 "ms" 等时间单位
static raw_t raw;

class serial_imu : public rclcpp::Node{
public:
    serial_imu() : Node("serial_imu_old"){
        this->declare_parameter<std::string>("dev", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<std::string>("topic", "/imu/data");
        IMU_pub = this->create_publisher<sensor_msgs::msg::Imu>(
            this->get_parameter("topic").as_string(), 20);

        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        sp.setPort(this->get_parameter("dev").as_string());
        sp.setBaudrate(this->get_parameter("baud_rate").as_int());
        sp.setTimeout(to);

        try{
            sp.open(); 
        }
        catch(serial::IOException& e){
            RCLCPP_ERROR(this->get_logger(), "Unable to open port");
            rclcpp::shutdown();
        }
        if(sp.isOpen()){
            RCLCPP_INFO(this->get_logger(), "Serial Port is opened"); 
        }
        else{
            rclcpp::shutdown();     
        }
        time_ = this->create_wall_timer(8ms, std::bind(&serial_imu::callback, this));
    }

private:

    void callback(){
        int rev=0;
        size_t num=sp.available();
        if(num!=0){
            uint8_t buffer[BUF_SIZE];
            if(num>BUF_SIZE)num=BUF_SIZE;

            num=sp.read(buffer, num);
            if(num>0){
                imu_data.header.stamp=this->now();
                imu_data.header.frame_id="imu_link";

                for(int i=0; i<num; i++){
                    rev=ch_serial_input(&raw, buffer[i]);
                    if(raw.item_code[raw.nitem_code-1] != KItemGWSOL){
                        if(rev){
                            publish_imu_data(&raw, &imu_data);
                            IMU_pub->publish(imu_data); 
                        } 
                    }
                }
            
            } 
        } 
    }

    void publish_imu_data(raw_t *data, sensor_msgs::msg::Imu *imu_data){
        imu_data->orientation.x=data->imu[data->nimu-1].quat[1];
        imu_data->orientation.y=data->imu[data->nimu-1].quat[2];
        imu_data->orientation.z=data->imu[data->nimu-1].quat[3];
        imu_data->orientation.w=data->imu[data->nimu-1].quat[0];
        imu_data->angular_velocity.x=data->imu[data->nimu-1].gyr[0] * DEG_TO_RAD;
        imu_data->angular_velocity.y=data->imu[data->nimu-1].gyr[1] * DEG_TO_RAD;
        imu_data->angular_velocity.z=data->imu[data->nimu-1].gyr[2] * DEG_TO_RAD;
        imu_data->linear_acceleration.x=data->imu[data->nimu-1].acc[0] * GRA_ACC;
        imu_data->linear_acceleration.y=data->imu[data->nimu-1].acc[1] * GRA_ACC;
        imu_data->linear_acceleration.z=data->imu[data->nimu-1].acc[2] * GRA_ACC;
        
    }
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr IMU_pub;
    serial::Serial sp;
    sensor_msgs::msg::Imu imu_data;
    rclcpp::TimerBase::SharedPtr time_;
};


int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<serial_imu>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}