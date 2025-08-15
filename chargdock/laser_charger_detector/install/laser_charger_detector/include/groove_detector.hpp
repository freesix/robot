#ifndef GROOVE_DETECTOR_HPP
#define GROOVE_DETECTOR_HPP

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
#include <limits>
#include <cmath>
#include <algorithm>
#include <vector>
#include <deque>

// 解决uchar未定义
using uchar = unsigned char;

namespace laser_charger_detector {
namespace srv {
struct Shutdown;  // 前向声明
}
}

class GrooveDetector : public rclcpp::Node
{
public:
    GrooveDetector();
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
    
    // V形模板匹配函数
    void generateVTemplate();
    double matchVTemplate(const std::vector<pcl::PointXYZ>& points, 
                         const pcl::PointXYZ& center, 
                         double rotation_angle);
    bool detectVGroove(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                       pcl::PointXYZ& groove_center,
                       double& best_angle,
                       std::vector<pcl::PointXYZ>& matched_points);
    
    // 订阅者和发布者
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detection_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr groove_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr curvature_points_pub_;
    
    // 点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr groove_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr curvature_cloud_;
    
    // 点云缓存
    std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcl_buf_;
    std::deque<builtin_interfaces::msg::Time> time_buf_;
    std::deque<std_msgs::msg::Header> header_buf_;
    const size_t max_buffer_size_ = 10;  // 最大缓存大小
    
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
    float max_range_ = 5.0;  // 距离限制在5米以内
    
    // V形模板匹配参数
    struct VTemplate {
        std::vector<pcl::PointXYZ> points;
        double width;           // V形开口宽度
        double depth;           // V形深度
        double angle;           // V形角度（弧度）
    };
    
    VTemplate v_template_;
    
    // 模板匹配参数
    double template_width_ = 0.245;        // V形模板宽度 24.5cm
    double template_depth_ = 0.0525;       // V形模板深度 5.25cm
    double template_angle_ = 3 * M_PI / 4; // V形角度 135度
    double match_threshold_ = 0.75;        // 匹配阈值（降低以适应更大模板）
    double rotation_step_ = M_PI / 36;     // 旋转步长 5度
    double translation_step_ = 0.015;      // 平移步长 1.5cm（适应更大模板）
    int template_points_num_ = 50;         // 模板点数量（增加到50个）
};

#endif // GROOVE_DETECTOR_HPP 