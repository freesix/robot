#ifndef CHASSIS_CONTROLLER_HPP
#define CHASSIS_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
#include <mutex>
#include <deque>
#include <memory>
#include <variant>
#include <control_toolbox/pid.hpp>
#include <control_toolbox/pid_ros.hpp>
#include "bezier_curve.hpp"

// 凹槽信息结构体
struct GrooveInfo {
    bool detected;          // 是否检测到凹槽
    double center_x;        // 凹槽中心点X坐标 (相对于机器人)
    double center_y;        // 凹槽中心点Y坐标 (相对于机器人)
    double normal_x;        // 正向法向量X分量
    double normal_y;        // 正向法向量Y分量
    double forward_x;       // 前向点X坐标
    double forward_y;       // 前向点Y坐标
};

// V形凹槽特征缓存结构体
struct GrooveFeatures {
    double center_x, center_y;
    double normal_x, normal_y;
    double forward_x, forward_y;
};

// 速度指令结构体
struct VelocityCommand {
    double linear_x;
    double linear_y;
    double angular_z;
};

// 底盘控制状态枚举
enum class ChassisState {
    STOP,           // 停止状态
    DETECT_GROOVE,  // 识别凹槽状态
    MOVING,         // 移动状态
    FINE_TUNING,    // 精细调整状态
    COMPLETED       // 完成状态
};

// 节点运行状态枚举
enum class NodeState {
    SLEEP,          // 休眠状态
    ACTIVE          // 激活状态
};

// 修改为仅继承rclcpp::Node，不再使用enable_shared_from_this
class ChassisController : public rclcpp::Node
{
public:
    ChassisController();
    ~ChassisController();
    
    // 添加初始化方法的声明
    void initialize();

private:
    // 服务回调函数
    void startTaskCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                          std_srvs::srv::Trigger::Response::SharedPtr response);
    void sleepNodeCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                          std_srvs::srv::Trigger::Response::SharedPtr response);
    void getStatusCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                          std_srvs::srv::Trigger::Response::SharedPtr response);
    
    // 节点状态管理函数
    bool isNodeActive();
    
    // 核心状态机函数
    void run();
    void executeState();
    void transitionToState(ChassisState new_state);
    
    // 状态处理函数
    void handleStopState();
    void handleDetectGrooveState();
    void handleMovingState();
    void handleFineTuningState();
    void handleCompletedState();
    
    // 运动控制函数
    void publishVelocity(double linear_x, double linear_y, double angular_z);
    void stopRobot();
    void pathTrackingCallback();
    
    // 直线移动控制函数
    void startDirectMovement();
    
    // 速度指令序列函数
    void generateVelocityCommands();
    void executeVelocityCommands();
    
    // 回调函数
    void groovePointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    // 点云处理函数
    void calculateVGrooveFeatures();
    void updateAverageGrooveInfo();
    
    // 工具函数
    std::string stateToString(ChassisState state);
    void logStateTransition(ChassisState from, ChassisState to);
    
    // 坐标转换函数
    void transformLidarToBody(double lidar_x, double lidar_y, double& body_x, double& body_y);
    
    // 里程计相关函数
    bool getCurrentPose(double& x, double& y, double& yaw);
    
    // ROS2 发布者和订阅者
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr groove_vis_pub_;  // 凹槽可视化发布者
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr groove_points_sub_;
    
    // 服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_task_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr sleep_node_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_status_service_; // 新增状态查询服务
    
    // 定时器
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr path_tracking_timer_;
    
    // TF相关
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string odom_frame_;
    std::string base_frame_;
    double target_yaw_angle_1;
    
    // 节点状态管理
    NodeState node_state_;
    mutable std::mutex node_state_mutex_;
    
    // 状态机变量
    ChassisState current_state_;
    ChassisState previous_state_;
    
    // 状态时间记录
    rclcpp::Time state_start_time_;
    
    // 传感器数据
    std::mutex data_mutex_;
    GrooveInfo groove_info_;        // 凹槽信息
    geometry_msgs::msg::Twist last_commanded_velocity_; // 上一次发布的速度指令
    bool task_started_;
    bool coarse_movement_completed_;  // 粗略移动完成标志位
    std::chrono::steady_clock::time_point path_tracking_start_time_;
    std::chrono::steady_clock::time_point path_tracking_last_log_time_;
    
    // 点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pcl_cloud_;  // 当前点云数据
    rclcpp::Time last_pointcloud_time_;                       // 上一帧点云数据的时间
    rclcpp::Time current_pointcloud_time_;                    // 当前帧点云数据的时间
    
    // V形凹槽特征缓存
    std::deque<GrooveFeatures> groove_features_buffer_;
    static const size_t MAX_GROOVE_FEATURES_BUFFER_SIZE = 10;
    
    // 控制参数
    double default_linear_speed_;       // 默认线速度 (m/s)
    double align_speed_;               // 对齐速度 (m/s)
    double control_frequency_;          // 控制频率 (Hz)
    // 超时参数
    double max_detect_time_;            // 最大识别时间 (s)
    double max_moving_time_;            // 最大移动时间 (s)
    
    // 里程计直线移动相关
    double start_x_, start_y_, start_yaw_;          // 起始位置
    bool start_pose_recorded_;                      // 是否记录了起始位置
    bool path_tracking_started_;                    // 直线移动是否已开始
    bool target_point_set_;                         // 目标点是否已设置
    BezierCurve::Point2D target_point_;             // 目标点
    
    // 固定步长路径执行参数
    std::vector<VelocityCommand> velocity_commands_; // 速度指令序列
    size_t current_command_index_;                   // 当前指令索引
    double command_step_time_;                       // 指令步长时间
    bool velocity_commands_generated_;               // 速度指令是否已生成
    
    // PID控制相关 - 差速机器人模型
    std::shared_ptr<control_toolbox::PidROS> pid_linear_x_;    // 线速度PID控制器
    std::shared_ptr<control_toolbox::PidROS> pid_angular_z_;    // 角速度PID控制器
    
    // 精细调整阶段专用PID控制器
    // 已删除精细调整X方向线速度PID控制器，改为固定速度和步数控制
    std::shared_ptr<control_toolbox::PidROS> pid_fine_angular_z_;  // 精细调整角速度PID控制器
    
    // PID控制器初始化函数
    void initializePIDControllers();
    void initializeFineTuningPIDControllers();
    
    // 状态标志
    bool is_initialized_;
    bool verbose_logging_;
};

#endif // CHASSIS_CONTROLLER_HPP 