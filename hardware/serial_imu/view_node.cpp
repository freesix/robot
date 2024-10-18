#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class ImuTfBroadcaster : public rclcpp::Node {
public:
    ImuTfBroadcaster() : Node("imu_tf_broadcaster") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10, std::bind(&ImuTfBroadcaster::imu_callback, this, std::placeholders::_1)
        );
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = msg->header.stamp;
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "imu_link";

        // 将 IMU 姿态设置为变换
        transformStamped.transform.rotation = msg->orientation;

        // 发布 TF 变换
        tf_broadcaster_->sendTransform(transformStamped);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuTfBroadcaster>());
    rclcpp::shutdown();
    return 0;
}