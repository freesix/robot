#ifndef GR_DETECTOR2_HPP
#define GR_DETECTOR2_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Dense>
#include <mutex>
#include <vector>
#include <cmath>
#include <algorithm>

// 解决uchar未定义问题
using uchar = unsigned char;

class GrDetector2 : public rclcpp::Node
{
public:
    GrDetector2();
    void run();

private:
    // 回调函数
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    
    // 处理函数
    bool syncData();
    void detectGroove();
    void publishResults();
    
    // 辅助函数
    bool lineIntersection(const cv::Point2f& p1, const cv::Point2f& p2, 
                          const cv::Point2f& p3, const cv::Point2f& p4,
                          cv::Point2f& intersection);
    bool isPointOnLineSegment(const cv::Point2f& p, const cv::Point2f& start, 
                              const cv::Point2f& end, float tolerance = 1.0f);
    float pointToLineSegmentDistance(const pcl::PointXYZ& point, 
                                    const pcl::PointXYZ& line_start, 
                                    const pcl::PointXYZ& line_end);
    
    // 订阅者和发布者
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detection_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr groove_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_points_pub_;
    
    // 点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr groove_cloud_;
    
    // 状态变量
    std::mutex cloud_mutex_;
    bool new_data_available_ = false;
    bool first_frame_processed_ = false;
    bool groove_detected_ = false;
    
    // 时间戳和坐标系
    builtin_interfaces::msg::Time current_scan_time_;
    std_msgs::msg::Header current_scan_header_;
    
    // 参数
    float min_range_ = 0.1;    // 最小有效距离
    float max_range_ = 10.0;   // 最大有效距离
    
    // 凹槽检测参数
    float neighborhood_radius_ = 0.1;     // 邻域搜索半径
    int min_neighborhood_size_ = 5;       // 最小邻域点数
    int min_line_points_ = 5;             // 最小拟合线段点数
    float line_distance_threshold_ = 0.01; // 点到线距离阈值
    float v_shape_min_score_ = 0.5;       // V形最低得分阈值
    float min_groove_angle_ = 30.0;       // 最小凹槽角度(度)
    float max_groove_angle_ = 150.0;      // 最大凹槽角度(度)
};

#endif // GR_DETECTOR2_HPP