#include "zfy_imu_node/imu_handle.hpp"
#include <memory>
#include <Eigen/Eigen>

#define G 9.80665
std::string dev;
uint32_t baud;
std::string topic;
std::string imu_frame;
void imu2ros(sensor_msgs::msg::Imu &imu, imu_data &data);

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("imu_node");
    node->declare_parameter("dev", "/dev/ttyUSB0");
    node->declare_parameter("baud_rate", 115200);
    node->declare_parameter("topic", "/imu/data");
    node->declare_parameter("imu_link", "imu_link");

    dev = node->get_parameter("dev").as_string();
    baud = node->get_parameter("baud_rate").as_int();
    topic = node->get_parameter("topic").as_string();
    imu_frame = node->get_parameter("imu_link").as_string();

    auto pub_ = node->create_publisher<sensor_msgs::msg::Imu>(topic, 10);
    
    imu_handle imuHandle(dev, baud);
    rclcpp::sleep_for(std::chrono::seconds(1));  
        
    if(imuHandle.isOpen()){
        RCLCPP_INFO_STREAM(node->get_logger(), "Open Serial "<<dev<<" is Successful!");
    }
    else{
        RCLCPP_INFO(node->get_logger(), "Can not open Serial");
        return -1;
    }
    raw_data *rawdata;
    sensor_msgs::msg::Imu Imu_;
    imu_data imudata;
    while(rclcpp::ok()){
        size_t num = imuHandle.available();
        if(num > 0){
            uint8_t buffer[BUF_SIZE];
            if(num > BUF_SIZE){
                num = BUF_SIZE;
            }
            num = imuHandle.read(buffer, num);
            if(num > 0){
                bool rev = imuHandle.decode(imudata, buffer); 
                if(rev == false){
                    continue;
                }
                imu2ros(Imu_, imudata);
                pub_->publish(Imu_);
            } 
        }
    }

    rclcpp::shutdown();
    return 0;
}

void imu2ros(sensor_msgs::msg::Imu &imu, imu_data &data){
    imu.header.stamp = rclcpp::Clock().now();
    imu.header.frame_id = imu_frame;
    imu.linear_acceleration.x = data.acc[0] * G;
    imu.linear_acceleration.y = data.acc[1] * G;
    imu.linear_acceleration.z = data.acc[2] * G;
    imu.angular_velocity.x = data.gyr[0];
    imu.angular_velocity.y = data.gyr[1];
    imu.angular_velocity.z = data.gyr[2];
    imu.orientation.w = data.quat[0];
    imu.orientation.x = data.quat[1];
    imu.orientation.y = data.quat[2];
    imu.orientation.z = data.quat[3];
}