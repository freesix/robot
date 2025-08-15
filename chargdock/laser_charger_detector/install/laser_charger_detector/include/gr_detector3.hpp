#ifndef GR_DETECTOR3_HPP
#define GR_DETECTOR3_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <builtin_interfaces/msg/time.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>


// 解决uchar未定义
using uchar = unsigned char;

namespace laser_charger_detector {
namespace srv {
struct Shutdown;  // 前向声明
}
}

class ChargerDetector : public rclcpp::Node
{
public:
    ChargerDetector();
    void run();

private:
    // 回调函数
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    
    // 数据处理函数
    bool syncData();
    void detectGroove();
    void publishResults();
    
    // RANSAC直线拟合函数
    bool ransacLineFit(const std::vector<pcl::PointXYZ>& points, 
                      std::vector<pcl::PointXYZ>& inliers,
                      pcl::PointXYZ& line_start, pcl::PointXYZ& line_end);
    
    // 订阅者和发布者
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detection_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr groove_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_points_pub_;
    
    
    // 点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr groove_cloud_;
    
    // 激光数据相关参数
    std::mutex cloud_mutex_;
    builtin_interfaces::msg::Time current_scan_time_;
    std_msgs::msg::Header current_scan_header_;
    bool new_data_available_ = false;
    bool first_frame_processed_;
    bool groove_detected_;
    
    // 激光扫描参数
    int scan_start_idx_ = 0;
    int scan_end_idx_ = 1080;
    float min_range_ = 0.1;
    float max_range_ = 10.0;
};

#endif // GR_DETECTOR3_HPP 