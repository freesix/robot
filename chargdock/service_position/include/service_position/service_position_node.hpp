#pragma once

#include "rclcpp/rclcpp.hpp"
#include "service_position_msgs/srv/get_position.hpp"
#include "service_position_msgs/srv/set_position.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// V形凹槽特征结构体
struct GrooveFeatures {
    double center_x;
    double center_y;
    double normal_x;
    double normal_y;
    double forward_x;
    double forward_y;
};

class ServicePositionNode : public rclcpp::Node {
public:
    ServicePositionNode();
private:
    rclcpp::Service<service_position_msgs::srv::SetPosition>::SharedPtr set_position_srv_;
    rclcpp::Service<service_position_msgs::srv::GetPosition>::SharedPtr get_position_srv_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr groove_points_sub_;
    
    // TF2相关
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 存储当前位置和点云数据
    geometry_msgs::msg::Pose2D current_position_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pcl_cloud_;
    GrooveFeatures current_groove_features_;
    
    // URDF偏移量
    double urdf_x_;
    double urdf_y_;
    
    void handle_set_position(const std::shared_ptr<service_position_msgs::srv::SetPosition::Request> request,
                            std::shared_ptr<service_position_msgs::srv::SetPosition::Response> response);
    void handle_get_position(const std::shared_ptr<service_position_msgs::srv::GetPosition::Request> request,
                            std::shared_ptr<service_position_msgs::srv::GetPosition::Response> response);
    void groovePointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void calculateVGrooveFeatures();
    bool transformPointToMap(double x, double y, double& map_x, double& map_y);
}; 