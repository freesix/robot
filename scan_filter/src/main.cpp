#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserScanFilter : public rclcpp::Node
{
public:
    LaserScanFilter() : Node("laser_scan_filter")
    {
        // 订阅激光扫描话题
        laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/laser", 10, std::bind(&LaserScanFilter::laser_scan_callback, this, std::placeholders::_1));

        // 创建发布者
        laser_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/laser/data", 10);

        // 初始化前一帧的时间戳
        last_timestamp_ = rclcpp::Time(0);
    }

private:
    // 订阅回调函数
    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 如果当前时间戳和上一帧时间戳相同，丢弃当前帧
        if (msg->header.stamp == last_timestamp_)
        {
            // RCLCPP_INFO(this->get_logger(), "Time stamp is the same as the previous frame, discarding this frame.");
            return;
        }

        // 如果时间戳不同，发布上一帧并更新时间戳
        last_timestamp_ = msg->header.stamp;

        // 发布滤波后的激光扫描消息
        laser_scan_publisher_->publish(*msg);

        // RCLCPP_INFO(this->get_logger(), "Published a new LaserScan message with timestamp: %ld", last_timestamp_.nanoseconds());
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;
    rclcpp::Time last_timestamp_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanFilter>());
    rclcpp::shutdown();
    return 0;
}
