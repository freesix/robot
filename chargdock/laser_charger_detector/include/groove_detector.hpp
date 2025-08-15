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
#include <pcl/surface/mls.h>
#include <pcl/filters/bilateral.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <builtin_interfaces/msg/time.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <limits>
#include <cmath>
#include <algorithm>
#include <vector>
#include <deque>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

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
    struct BBSTarget {
        int level;              // 层级
        int cloud_index;        // 点云索引
        double rotation_angle;  // 旋转角度
        double score;           // 分数
    };

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
    double matchVTemplate(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_cloud, 
                         const std::vector<pcl::PointXYZ>& template_points);
     
    // BBS得分计算函数
    double calculateScore(const BBSTarget& target);
    
    // 订阅者和发布者
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detection_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr groove_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_points_pub_;
    
    // 服务服务器
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_detection_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_detection_srv_;
    
    // 服务回调函数
    void startDetectionCallback(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response);
    
    void stopDetectionCallback(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response);

    
    // 点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr groove_cloud_;
    
    // KD树用于近邻搜索
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_;
    
    // 激光数据相关参数
    std::mutex cloud_mutex_;
    builtin_interfaces::msg::Time current_scan_time_;
    std_msgs::msg::Header current_scan_header_;
    bool new_data_available_ = false;
    bool first_frame_processed_;
    bool groove_detected_;
    bool is_active_;  // 休眠状态标志
    
    // 最佳匹配结果
    pcl::PointXYZ best_groove_center_;
    double best_groove_angle_;
    
    // BBS优先队列
    std::priority_queue<BBSTarget, std::vector<BBSTarget>, 
                       std::function<bool(const BBSTarget&, const BBSTarget&)>> bbs_queue_;
    
    // 激光扫描参数
    float min_range_ = 0.1;
    float max_range_ = 3.0;  // 距离限制在5米以内
    
    // 强度过滤参数
    bool enable_intensity_filter_ = true;   // 是否启用强度过滤
    float min_intensity_ = 100.0;            // 最小强度阈值
    float max_intensity_ = 200.0;          // 最大强度阈值
    
    // 拖尾滤波参数
    double trailing_filter_alpha_ = 0.3;  // 指数平滑系数
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud_;  // 前一帧点云
    
    // 平滑滤波参数
    double search_radius_ = 0.03;  // MLS搜索半径 3cm
    int polynomial_order_ = 2;     // 多项式拟合阶数
    
    // V形模板匹配参数
    
    struct VTemplate {
        std::vector<pcl::PointXYZ> points;
        double width;           // V形开口宽度
        double depth;           // V形深度
        double angle;           // V形角度（弧度）
    };
    
    VTemplate v_template_;
    
    // 模板匹配参数
    double template_edge_length_ = 0.135;  // V形边长 13.5cm
    double template_angle_ = 136.5 * M_PI / 180.0; // V形角度 136.5度
    double match_threshold_ = 0.60;        // 匹配阈值
    double rotation_step_ = M_PI / 36;     // 旋转步长 5度
    int template_points_num_ = 60;         // 模板点数量

};

#endif // GROOVE_DETECTOR_HPP 