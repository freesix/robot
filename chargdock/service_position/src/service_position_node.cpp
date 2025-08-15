#include "service_position/service_position_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <sstream>
#include <filesystem>
#include <regex>
#include <cstdlib>
#include <tf2/LinearMath/Quaternion.h>
#include <cstdio> // For printf
#include <cmath>  // For cos, sin, atan2

ServicePositionNode::ServicePositionNode() : Node("service_position_node") {
    RCLCPP_INFO(this->get_logger(), "ServicePositionNode 启动");
    
    // 初始化TF2
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    
    // 初始化当前位置和点云数据
    current_position_.x = 0.0;
    current_position_.y = 0.0;
    current_pcl_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    
    // 初始化URDF偏移量
    urdf_x_ = 0.065;
    urdf_y_ = 0.0;
    
    // 订阅groove_points话题
    groove_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "groove_points", 10, 
        std::bind(&ServicePositionNode::groovePointsCallback, this, std::placeholders::_1)
    );
    
    set_position_srv_ = this->create_service<service_position_msgs::srv::SetPosition>(
        "set_position",
        std::bind(&ServicePositionNode::handle_set_position, this, std::placeholders::_1, std::placeholders::_2)
    );
    get_position_srv_ = this->create_service<service_position_msgs::srv::GetPosition>(
        "get_position",
        std::bind(&ServicePositionNode::handle_get_position, this, std::placeholders::_1, std::placeholders::_2)
    );
}

void ServicePositionNode::groovePointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 将ROS消息转换为PCL点云
    pcl::fromROSMsg(*msg, *current_pcl_cloud_);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "接收到点云数据，大小: %zu", current_pcl_cloud_->points.size());
    
}

void ServicePositionNode::calculateVGrooveFeatures() {
    if (!current_pcl_cloud_ || current_pcl_cloud_->points.empty()) {
        return;
    }
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "开始计算V形凹槽特征，点云大小: %zu", current_pcl_cloud_->points.size());
    
    // 检查点云数据是否足够
    if (current_pcl_cloud_->points.size() < 10) {
        RCLCPP_WARN(this->get_logger(), "点云数据不足，跳过本帧");
        return;
    }
    
    const auto& vertex_point = current_pcl_cloud_->points[0];
    double center_x = vertex_point.x;
    double center_y = vertex_point.y;
    std::cout<<center_x<<","<<center_y<<std::endl;

    double avg_x = 0.0;
    double avg_y = 0.0;
    
    for (const auto& point : current_pcl_cloud_->points) {
        avg_x += point.x;
        avg_y += point.y;
    }
    
    avg_x /= current_pcl_cloud_->points.size();
    avg_y /= current_pcl_cloud_->points.size();
    std::cout<<avg_x<<","<<avg_y<<std::endl;
 
    double bisector_x = avg_x - center_x;
    double bisector_y = avg_y - center_y;
    
    double bisector_length = sqrt(bisector_x*bisector_x + bisector_y*bisector_y);
    double normal_x = bisector_x / bisector_length;
    double normal_y = bisector_y / bisector_length;
    
    double forward_distance = 0.8;
    double forward_x = center_x + normal_x * forward_distance;
    double forward_y = center_y + normal_y * forward_distance;
    
    current_groove_features_.center_x = center_x;
    current_groove_features_.center_y = center_y;
    current_groove_features_.normal_x = normal_x;
    current_groove_features_.normal_y = normal_y;
    current_groove_features_.forward_x = forward_x;
    current_groove_features_.forward_y = forward_y;
    
    RCLCPP_INFO(this->get_logger(), "V形凹槽特征计算完成: forward_x=%.3f, forward_y=%.3f", forward_x, forward_y);
}

bool ServicePositionNode::transformPointToMap(double x, double y, double& map_x, double& map_y) {
    try {

        geometry_msgs::msg::PointStamped point_base;
        point_base.header.frame_id = "base_link";
        // 使用当前时间，让TF2自动处理时间戳
        point_base.header.stamp = rclcpp::Time(0);
        point_base.point.x = x;
        point_base.point.y = y;
        point_base.point.z = 0.0;
        
        // 转换到map坐标系
        geometry_msgs::msg::PointStamped point_map;
        point_map = tf_buffer_->transform(point_base, "map");
        
        map_x = point_map.point.x;
        map_y = point_map.point.y;
        
        RCLCPP_INFO(this->get_logger(), "坐标转换成功: map(%.3f, %.3f)", 
                   map_x, map_y);
        return true;
    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "坐标转换失败: %s", ex.what());
        return false;
    }
}


void euler_to_quaternion(double roll, double pitch, double yaw, double &qx, double &qy, double &qz, double &qw) {
    // 打印输入的欧拉角
    printf("输入欧拉角: roll=%.6f, pitch=%.6f, yaw=%.6f\n", roll, pitch, yaw);
    // 计算四元数
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    
    qw = cr * cp * cy + sr * sp * sy;
    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;
}

void ServicePositionNode::handle_set_position(
    const std::shared_ptr<service_position_msgs::srv::SetPosition::Request> request,
    std::shared_ptr<service_position_msgs::srv::SetPosition::Response> response)
{
    // 在服务调用时计算V形凹槽特征
    calculateVGrooveFeatures();
    
    // 获取用户主目录并构建文件路径
    const char* home_dir = std::getenv("HOME");
    if (!home_dir) {
        response->success = false;
        response->message = "无法获取用户主目录";
        RCLCPP_ERROR(this->get_logger(), "无法获取HOME环境变量");
        return;
    }
    
    std::string filename = std::string(home_dir) + "/position.xml";
    RCLCPP_INFO(this->get_logger(), "准备写入文件: %s", filename.c_str());
    
    std::ofstream ofs(filename, std::ios::out | std::ios::trunc);
    if (!ofs.is_open()) {
        response->success = false;
        response->message = "无法打开文件";
        RCLCPP_ERROR(this->get_logger(), "无法打开文件: %s", filename.c_str());
        return;
    }
    
    // 使用计算出的forward点作为位置信息
    double save_x = current_groove_features_.forward_x - urdf_x_;
    double save_y = current_groove_features_.forward_y - urdf_y_;
    RCLCPP_INFO(this->get_logger(), "base_link(%.3f, %.3f)", 
            save_x, save_y);
    // 转换到map坐标系
    double map_x, map_y;
    if (!transformPointToMap(save_x, save_y, map_x, map_y)) {
        response->success = false;
        response->message = "坐标转换失败";
        RCLCPP_ERROR(this->get_logger(), "坐标转换失败，无法保存位置");
        return;
    }
    // 计算朝向：将center点转换到baselink坐标系
    double center_baselink_x = current_groove_features_.center_x - urdf_x_;
    double center_baselink_y = current_groove_features_.center_y - urdf_y_;
        RCLCPP_INFO(this->get_logger(), "base_link(%.3f, %.3f)", 
            center_baselink_x, center_baselink_y);
        // 将朝向向量转换到map坐标系
    double map_center_x, map_center_y;
    if (!transformPointToMap(center_baselink_x, center_baselink_y, map_center_x, map_center_y)) {
        response->success = false;
        response->message = "朝向转换失败";
        RCLCPP_ERROR(this->get_logger(), "朝向转换失败，无法保存朝向");
        return;
    }  
    double direction_x = map_center_x - map_x;
    double direction_y = map_center_y - map_y;
    // 将朝向向量转换为四元数（绕Z轴旋转）
    double yaw = atan2(direction_y, direction_x);
    double qx, qy, qz, qw;
    euler_to_quaternion(0.0, 0.0, yaw, qx, qy, qz, qw);
    
    // 保存转换后的坐标和朝向到XML文件
    ofs << "<position>\n";
    ofs << "  <x>" << map_x << "</x>\n";
    ofs << "  <y>" << map_y << "</y>\n";
    ofs << "  <orientation_x>" << qx << "</orientation_x>\n";
    ofs << "  <orientation_y>" << qy << "</orientation_y>\n";
    ofs << "  <orientation_z>" << qz << "</orientation_z>\n";
    ofs << "  <orientation_w>" << qw << "</orientation_w>\n";
    ofs << "</position>\n";
    
    // 检查写入是否成功
    if (ofs.fail()) {
        response->success = false;
        response->message = "文件写入失败";
        RCLCPP_ERROR(this->get_logger(), "文件写入失败: %s", filename.c_str());
        ofs.close();
        return;
    }
    
    ofs.close();
    
    // 验证文件是否真的被写入
    std::ifstream check_file(filename);
    if (!check_file.is_open()) {
        response->success = false;
        response->message = "文件写入验证失败";
        RCLCPP_ERROR(this->get_logger(), "无法验证文件写入: %s", filename.c_str());
        return;
    }
    
    std::string content((std::istreambuf_iterator<char>(check_file)), std::istreambuf_iterator<char>());
    check_file.close();
    
    RCLCPP_INFO(this->get_logger(), "文件内容验证: %s", content.c_str());
    
    response->success = true;
    response->message = "保存成功";
    RCLCPP_INFO(this->get_logger(), "已保存map坐标系下的forward点x=%.3f, y=%.3f到%s", map_x, map_y, filename.c_str());
}

void ServicePositionNode::handle_get_position(
    const std::shared_ptr<service_position_msgs::srv::GetPosition::Request> /*request*/,
    std::shared_ptr<service_position_msgs::srv::GetPosition::Response> response)
{
    // 获取用户主目录并构建文件路径
    const char* home_dir = std::getenv("HOME");
    if (!home_dir) {
        response->success = false;
        response->message = "无法获取用户主目录";
        response->x = 0.0;
        response->y = 0.0;
        RCLCPP_ERROR(this->get_logger(), "无法获取HOME环境变量");
        return;
    }
    
    std::string filename = std::string(home_dir) + "/position.xml";
    
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        response->success = false;
        response->message = "文件不存在";
        response->x = 0.0;
        response->y = 0.0;
        RCLCPP_WARN(this->get_logger(), "文件不存在: %s", filename.c_str());
        return;
    }
    
    std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    ifs.close();
    
    std::smatch match_x, match_y, match_ox, match_oy, match_oz, match_ow;
    std::regex regex_x("<x>([-+]?[0-9]*\.?[0-9]+)</x>");
    std::regex regex_y("<y>([-+]?[0-9]*\.?[0-9]+)</y>");
    std::regex regex_ox("<orientation_x>([-+]?[0-9]*\.?[0-9]+)</orientation_x>");
    std::regex regex_oy("<orientation_y>([-+]?[0-9]*\.?[0-9]+)</orientation_y>");
    std::regex regex_oz("<orientation_z>([-+]?[0-9]*\.?[0-9]+)</orientation_z>");
    std::regex regex_ow("<orientation_w>([-+]?[0-9]*\.?[0-9]+)</orientation_w>");
    
    if (std::regex_search(content, match_x, regex_x) && std::regex_search(content, match_y, regex_y)) {
        current_position_.x = std::stod(match_x[1]);
        current_position_.y = std::stod(match_y[1]);
        response->x = current_position_.x;
        response->y = current_position_.y;
        
        // 读取四元数信息
        if (std::regex_search(content, match_ox, regex_ox) && 
            std::regex_search(content, match_oy, regex_oy) &&
            std::regex_search(content, match_oz, regex_oz) &&
            std::regex_search(content, match_ow, regex_ow)) {
            response->orientation_x = std::stod(match_ox[1]);
            response->orientation_y = std::stod(match_oy[1]);
            response->orientation_z = std::stod(match_oz[1]);
            response->orientation_w = std::stod(match_ow[1]);
            RCLCPP_INFO(this->get_logger(), "读取到x=%.3f, y=%.3f, 四元数(%.3f, %.3f, %.3f, %.3f)", 
                       current_position_.x, current_position_.y,
                       response->orientation_x, response->orientation_y, 
                       response->orientation_z, response->orientation_w);
        } else {
            // 如果没有四元数信息，使用默认值
            response->orientation_x = 0.0;
            response->orientation_y = 0.0;
            response->orientation_z = 0.0;
            response->orientation_w = 1.0;
            RCLCPP_INFO(this->get_logger(), "读取到x=%.3f, y=%.3f, 使用默认四元数", 
                       current_position_.x, current_position_.y);
        }
        
        response->success = true;
        response->message = "读取成功";
    } else {
        response->success = false;
        response->message = "XML格式错误";
        response->x = 0.0;
        response->y = 0.0;
        response->orientation_x = 0.0;
        response->orientation_y = 0.0;
        response->orientation_z = 0.0;
        response->orientation_w = 1.0;
        RCLCPP_ERROR(this->get_logger(), "XML格式错误: %s", filename.c_str());
    }
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServicePositionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 