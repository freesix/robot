#include "chassis_controller.hpp"
#include <cmath>
#include <chrono>
#include <thread>

ChassisController::ChassisController()
    : Node("chassis_controller"),
      current_state_(ChassisState::STOP),
      previous_state_(ChassisState::STOP),
      task_started_(false),
      is_initialized_(false),
      verbose_logging_(true),
      node_state_(NodeState::SLEEP)  // 初始化为休眠状态
{
    // 只在构造函数中保留基本参数设置和成员变量初始化
    // 初始化参数
    control_frequency_ = 20.0;          // 20Hz
    
    // 超时参数
    max_detect_time_ = 30.0;            // 30秒
    max_moving_time_ = 20.0;            // 20秒
    
    // 里程计路径跟踪参数
    start_pose_recorded_ = false;
    start_x_ = 0.0;
    start_y_ = 0.0;
    start_yaw_ = 0.0;
    path_tracking_started_ = false;
    odom_frame_ = "odom";
    base_frame_ = "base_link";
    
    // 移除所有TF、发布者、订阅者、定时器相关代码
    
    // 初始化凹槽信息
    groove_info_.detected = false;
    groove_info_.center_x = 0.0;
    groove_info_.center_y = 0.0;
    groove_info_.normal_x = 0.0;
    groove_info_.normal_y = 0.0;
    groove_info_.forward_x = 0.0;
    groove_info_.forward_y = 0.0;
    
    // 初始化标志位
    coarse_movement_completed_ = false;

    // 初始化点云数据 
    current_pcl_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    
    // 初始化特征缓存 
    groove_features_buffer_.clear();
}

ChassisController::~ChassisController()
{
    // 确保所有定时器被取消
    if (path_tracking_timer_) {
        path_tracking_timer_->cancel();
        path_tracking_timer_.reset();  // 释放智能指针
    }
    
    // 析构时确保机器人停止
    stopRobot();
}

void ChassisController::run()
{
    if (!is_initialized_) {
        return;
    }
    
    // 检查节点是否处于激活状态
    if (!isNodeActive()) {
        return;  // 如果节点处于休眠状态，直接返回
    }
    
    // 自动启动任务
    if (!task_started_ && current_state_ == ChassisState::STOP) {
        task_started_ = true;
        transitionToState(ChassisState::DETECT_GROOVE);
    }
    
    // 执行当前状态
    executeState();
}

void ChassisController::executeState()
{
    switch (current_state_) {
        case ChassisState::STOP:
            handleStopState();
            break;
        case ChassisState::DETECT_GROOVE:
            handleDetectGrooveState();
            break;
        case ChassisState::MOVING:
            handleMovingState();
            break;
        case ChassisState::FINE_TUNING:
            handleFineTuningState();
            break;
        case ChassisState::COMPLETED:
            handleCompletedState();
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "未知状态: %d", static_cast<int>(current_state_));
            transitionToState(ChassisState::STOP);
            break;
    }
}

void ChassisController::handleStopState()
{
    // 停止状态 - 机器人静止
    stopRobot();
    
}

void ChassisController::handleDetectGrooveState()
{
    // 识别凹槽状态 - 机器人保持静止，等待凹槽检测结果
    stopRobot();

    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (current_pcl_cloud_ && !current_pcl_cloud_->points.empty()) {
            // 检查点云数据的时效性，确保是状态转换后的新数据
            auto time_since_pointcloud = last_pointcloud_time_ - current_pointcloud_time_;
            
            // 只有当点云数据是在状态开始之后接收的，或者点云很新鲜时才处理
            if (last_pointcloud_time_ > state_start_time_ || time_since_pointcloud.seconds() < 0.1) {
                // 计算V形凹槽的中心点和法向量
                calculateVGrooveFeatures();

                if (groove_info_.detected) {
                    // 创建一个点云对象用于可视化
                    pcl::PointCloud<pcl::PointXYZ>::Ptr groove_vis_cloud(new pcl::PointCloud<pcl::PointXYZ>());
                    pcl::PointXYZ p1, p2;
                    p1.x = groove_info_.center_x;
                    p1.y = groove_info_.center_y;
                    p1.z = 0;
                    p2.x = groove_info_.forward_x;
                    p2.y = groove_info_.forward_y;
                    p2.z = 0;
                    groove_vis_cloud->points.push_back(p1);
                    groove_vis_cloud->points.push_back(p2);
                    // 转换为ROS消息
                    sensor_msgs::msg::PointCloud2 groove_vis_msg;
                    pcl::toROSMsg(*groove_vis_cloud, groove_vis_msg);
                    groove_vis_msg.header.frame_id = "laser_link";
                    groove_vis_msg.header.stamp = this->now();

                    // 发布点云
                    groove_vis_pub_->publish(groove_vis_msg);
                }
            } else {
                // 点云数据过旧，等待新的数据
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                    "等待新的点云数据，当前数据时间差: %.3fs", 
                                    time_since_pointcloud.seconds());
            }
        }
    }
    
    // 检查是否检测到凹槽
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (groove_info_.detected) {
        if (coarse_movement_completed_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "检测到凹槽，开始精细调整");
            transitionToState(ChassisState::FINE_TUNING);
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "检测到凹槽，开始移动对齐");
            transitionToState(ChassisState::MOVING);
        }
    }
}

void ChassisController::handleMovingState()
{
    // 检查超时
    if (rclcpp::Clock().now()- state_start_time_ > rclcpp::Duration::from_seconds(max_moving_time_)) {
        RCLCPP_INFO(this->get_logger(), "移动对齐超时，任务完成");
        transitionToState(ChassisState::COMPLETED);
        return;
    }
    
    // 检查凹槽状态
    if (!groove_info_.detected) {
        RCLCPP_WARN(this->get_logger(), "凹槽丢失，返回检测状态");
        transitionToState(ChassisState::DETECT_GROOVE);
        return;
    }
    
    double current_x_1, current_y_1, current_yaw_1;
    if (!getCurrentPose(current_x_1, current_y_1, current_yaw_1)) {
        return;
    }

    if (!target_point_set_) {
        double forward_body_x, forward_body_y;
        transformLidarToBody(groove_info_.forward_x, groove_info_.forward_y, forward_body_x, forward_body_y);
        
        // 设置目标点
        target_point_.x = forward_body_x;
        target_point_.y = forward_body_y;
        
        RCLCPP_INFO(this->get_logger(), "设置直线移动目标点: (%.3f, %.3f)", target_point_.x, target_point_.y);
        target_point_set_ = true;
    }
    
    // 开始高频率直线移动（只在第一次进入时调用）
    if (!path_tracking_started_) {
        path_tracking_started_ = true;
        
        RCLCPP_INFO(this->get_logger(), "开始高频率直线移动 (20Hz)");
        startDirectMovement();
    }
}

void ChassisController::handleFineTuningState()
{
    // 精细调整状态的局部静态变量，用于状态管理
    static bool fine_tuning_initialized = false;
    static bool target_aligned = false;  // 标志位：是否已经对齐目标点
    static bool use_two_stage_adjustment = false;  // 标志位：是否使用二段式调整
    static double fine_tuning_start_x, fine_tuning_start_y, fine_tuning_start_yaw;
    static double target_projection_x, target_projection_y, target_yaw_angle;
    
    // 检查凹槽状态
    if (!groove_info_.detected) {
        RCLCPP_WARN(this->get_logger(), "凹槽丢失，返回检测状态");
        fine_tuning_initialized = false;
        target_aligned = false;  
        use_two_stage_adjustment = false;  
        transitionToState(ChassisState::DETECT_GROOVE);
        return;
    }
    
    if (!fine_tuning_initialized) {
        // 获取当前机器人位置作为起始位置
        if (!getCurrentPose(fine_tuning_start_x, fine_tuning_start_y, fine_tuning_start_yaw)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "无法获取起始位置，暂停精细调整");
            stopRobot();
            return;
        }
        
        // 将雷达坐标系下的点转换到机体坐标系
        double center_body_x, center_body_y;
        double forward_body_x, forward_body_y;
        
        transformLidarToBody(groove_info_.center_x, groove_info_.center_y, center_body_x, center_body_y);
        transformLidarToBody(groove_info_.forward_x, groove_info_.forward_y, forward_body_x, forward_body_y);
        center_body_x = center_body_x - 0.065;
        forward_body_x = forward_body_x -0.065;
        // 计算连线方向向量
        double line_dx = forward_body_x - center_body_x;
        double line_dy = forward_body_y - center_body_y;
        double line_length = sqrt(line_dx * line_dx + line_dy * line_dy);
        
        // 检查向量长度是否有效
        if (line_length < 0.001) {
            RCLCPP_WARN(this->get_logger(), "凹槽中心点和前向点距离太近，无法确定有效方向，使用默认方向");
            // 使用默认方向（X轴正方向）
            line_dx = 1.0;
            line_dy = 0.0;
            line_length = 1.0;
        }
        
        // 归一化连线方向向量
        double line_unit_x = line_dx / line_length;
        double line_unit_y = line_dy / line_length;
        
        double to_origin_x = -center_body_x;
        double to_origin_y = -center_body_y;
        
        double projection_length = to_origin_x * line_unit_x + to_origin_y * line_unit_y;
        
        target_projection_x = center_body_x + projection_length * line_unit_x;
        target_projection_y = center_body_y + projection_length * line_unit_y;
        
        // 计算目标航向角（机体坐标系下的目标角度）
        target_yaw_angle = atan2(line_unit_y, line_unit_x);
        target_yaw_angle_1= target_yaw_angle;
        // 计算初始状态下的误差（机体坐标系下，当前位置为原点）
        double initial_distance = sqrt(target_projection_x * target_projection_x + target_projection_y * target_projection_y);
        
        // 定义阈值
        const double distance_threshold = 0.065;  
        const double angle_threshold = 0.065;     
        
        RCLCPP_INFO(this->get_logger(), "精细调整初始化完成:");
        RCLCPP_INFO(this->get_logger(), "  起始位置: (%.3f, %.3f, %.3f度)", 
                   fine_tuning_start_x, fine_tuning_start_y, fine_tuning_start_yaw * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "  机体坐标系目标: (%.3f, %.3f, %.3f度)", 
                   target_projection_x, target_projection_y, target_yaw_angle);
        RCLCPP_INFO(this->get_logger(), "  初始误差: 距离=%.3f, 角度=%.3f", 
                   initial_distance, target_yaw_angle);
        
        // 检查初始状态是否已经满足条件
        if(initial_distance <= distance_threshold && fabs(target_yaw_angle) <= angle_threshold) {
            RCLCPP_INFO(this->get_logger(), "初始状态已满足条件，任务完成！");
            stopRobot();
            fine_tuning_initialized = false;
            target_aligned = false;  
            use_two_stage_adjustment = false;  
            transitionToState(ChassisState::COMPLETED);
            return;
        }
        
       
        if (initial_distance > distance_threshold) {
            use_two_stage_adjustment = true;  
            RCLCPP_INFO(this->get_logger(), "选择二段式调整策略（距离误差: %.3f > %.3f）", initial_distance, distance_threshold);
        } else {
            use_two_stage_adjustment = false;  
            RCLCPP_INFO(this->get_logger(), "选择角度调整策略（距离误差: %.3f <= %.3f）", initial_distance, distance_threshold);
        }
        
        fine_tuning_initialized = true;
        target_aligned = false;  
        return;  
    }
//--------------------------------------
    // 获取当前机器人位置 - 每次状态函数调用执行一次
    double current_x, current_y, current_yaw;
    if (!getCurrentPose(current_x, current_y, current_yaw)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "无法获取当前位置，暂停精细调整");
        stopRobot();
        transitionToState(ChassisState::DETECT_GROOVE);
        return; // 转换到检测状态
    }

    // 计算里程计相对于精细调整开始位置的变化
    double rel_x_odom = current_x - fine_tuning_start_x;
    double rel_y_odom = current_y - fine_tuning_start_y;
    
    // 将里程计坐标转换为精细调整开始时刻的坐标系（考虑精细调整开始时的航向角）
    double cos_yaw = cos(-fine_tuning_start_yaw);
    double sin_yaw = sin(-fine_tuning_start_yaw);
    double rel_x = rel_x_odom * cos_yaw - rel_y_odom * sin_yaw;
    double rel_y = rel_x_odom * sin_yaw + rel_y_odom * cos_yaw;
    
    double rel_yaw = current_yaw - fine_tuning_start_yaw;//朝向的变化量
    double position_error_x = target_projection_x - rel_x;
    double position_error_y = target_projection_y - rel_y;
    double distance_to_target = sqrt(position_error_x * position_error_x + position_error_y * position_error_y);
    target_yaw_angle = atan2(target_projection_y, target_projection_x);
    double yaw_error = target_yaw_angle - rel_yaw;
    double yaw_error_1 = target_yaw_angle_1 -rel_yaw;
    // 处理角度跳跃
    if (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    if (yaw_error < -M_PI) yaw_error += 2 * M_PI;
    if (yaw_error_1 > M_PI) yaw_error_1 -= 2 * M_PI;
    if (yaw_error_1 < -M_PI) yaw_error_1 += 2 * M_PI;
    // 定义阈值
    const double distance_threshold = 0.015;  
    const double angle_threshold = 0.06;     

    // 根据初始化时确定的策略执行控制
    if (use_two_stage_adjustment) {
        const double rotation_threshold = 0.05;  
        
        if (!target_aligned) {

            const double large_error_threshold = 0.3;    
            const double pid_transition_threshold = 0.4; 
            const double fixed_angular_speed = 0.4;      
            
            // 计算角度误差的绝对值
            double abs_yaw_error = fabs(yaw_error);
            int error_direction = (yaw_error > 0) ? 1 : -1;
            
            double angular_z = 0.0;
            
            // 混合控制策略：大角度用固定速度，小角度用PID
            if (abs_yaw_error > pid_transition_threshold) {
                // 大角度误差，使用固定角速度
                angular_z = fixed_angular_speed * error_direction;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                   "大角度旋转，使用固定角速度: %.3f (误差: %.3f)", angular_z, yaw_error);
            } 
            else if (abs_yaw_error > large_error_threshold) {
                // 过渡区域，线性插值以平滑过渡
                double pid_weight = (pid_transition_threshold - abs_yaw_error) / 
                                   (pid_transition_threshold - large_error_threshold);
                
                // 计算PID输出
                rclcpp::Duration dt = rclcpp::Duration::from_nanoseconds(50000000);
                double pid_output = pid_fine_angular_z_->computeCommand(yaw_error, dt);
                
                // 确保PID输出有效
                if (std::isnan(pid_output)) {
                    pid_output = 0.2 * error_direction;
                }
                
                // 线性插值混合固定速度和PID速度
                angular_z = (1.0 - pid_weight) * (fixed_angular_speed * error_direction) + 
                           pid_weight * pid_output;
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                   "过渡区域旋转，混合角速度: %.3f (误差: %.3f, PID权重: %.2f)", 
                                   angular_z, yaw_error, pid_weight);
            } 
            else {

                const double small_angle_speed = 0.15;  // 小角度时的固定角速度
                angular_z = small_angle_speed * error_direction;
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                   "精细调整旋转，固定角速度: %.3f (误差: %.3f)", angular_z, yaw_error);
            }
            
            publishVelocity(0.0, 0.0, angular_z);
            
            // 检查是否已经对齐目标点
            if (fabs(yaw_error) <= rotation_threshold) {
                target_aligned = true;
                RCLCPP_INFO(this->get_logger(), "目标点对齐完成，切换到直线运动模式");
            }
        } else {

            static int movement_counter = 0;
            static int total_movement_steps = 0;
            static double fixed_linear_speed = 0.0;
            static bool movement_initialized = false;
            
            // 初始化移动参数（只在第一次进入时执行）
            if (!movement_initialized) {
                double distance_to_move = distance_to_target;
            
                fixed_linear_speed = 0.05;
                
                double control_frequency = 20.0; // 假设控制循环为20Hz
                total_movement_steps = static_cast<int>(distance_to_move * control_frequency / fabs(fixed_linear_speed));
                
                // 确保最小移动时间
                const int min_steps = 40; // 至少2秒（20Hz * 2秒）
                total_movement_steps = std::max(total_movement_steps, min_steps);
                
                // 重置计数器
                movement_counter = 0;
                
                movement_initialized = true;
                
                RCLCPP_INFO(this->get_logger(), "初始化直线移动: 距离=%.3f米, 速度=%.3f m/s, 总步数=%d",
                           distance_to_move, fixed_linear_speed, total_movement_steps);
            }
            
            // 执行移动
            if (movement_counter < total_movement_steps) {
                // 发布固定速度
                publishVelocity(fixed_linear_speed, 0.0, 0.0);
                
                // 更新计数器
                movement_counter++;
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                   "直线运动进度: %d/%d (%.1f%%)", 
                                   movement_counter, total_movement_steps, 
                                   (movement_counter * 100.0) / total_movement_steps);
            } else {
                // 移动完成
                stopRobot();
                RCLCPP_INFO(this->get_logger(), "固定步数移动完成，返回检测状态重新评估");
                
                // 重置所有状态变量
                fine_tuning_initialized = false;
                target_aligned = false;
                use_two_stage_adjustment = false;
                movement_initialized = false; // 重置移动初始化标志
                
                transitionToState(ChassisState::DETECT_GROOVE);
                return;
            }
        }
    } else {
        const double large_error_threshold = 0.3;    
        const double pid_transition_threshold = 0.4; 
        const double fixed_angular_speed = 0.4;      
        
        // 计算角度误差的绝对值
        double abs_yaw_error = fabs(yaw_error_1);
        int error_direction = (yaw_error_1 > 0) ? 1 : -1;
        
        double angular_z = 0.0;
        
        // 混合控制策略：大角度用固定速度，小角度用PID
        if (abs_yaw_error > pid_transition_threshold) {
            // 大角度误差，使用固定角速度
            angular_z = fixed_angular_speed * error_direction;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                               "大角度调整，使用固定角速度: %.3f (误差: %.3f)", angular_z, yaw_error_1);
        } 
        else if (abs_yaw_error > large_error_threshold) {
            // 线性插值以平滑过渡
            double pid_weight = (pid_transition_threshold - abs_yaw_error) / 
                               (pid_transition_threshold - large_error_threshold);
            
            // 计算PID输出
            rclcpp::Duration dt = rclcpp::Duration::from_nanoseconds(50000000);
            double pid_output = pid_fine_angular_z_->computeCommand(yaw_error_1, dt);
            
            // 确保PID输出有效
            if (std::isnan(pid_output)) {
                pid_output = 0.2 * error_direction;
            }
            
            // 线性插值
            angular_z = (1.0 - pid_weight) * (fixed_angular_speed * error_direction) + 
                       pid_weight * pid_output;
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                               "过渡区域调整，混合角速度: %.3f (误差: %.3f, PID权重: %.2f)", 
                               angular_z, yaw_error_1, pid_weight);
        } 
        else {
            // 小角度误差，使用PID控制
            rclcpp::Duration dt = rclcpp::Duration::from_nanoseconds(50000000); // 固定10ms
            angular_z = pid_fine_angular_z_->computeCommand(yaw_error_1, dt);
            
            // 检查输出是否有效
            if (std::isnan(angular_z)) {
                RCLCPP_WARN(this->get_logger(), "PID输出为NaN，使用默认值");
                angular_z = 0.15 * error_direction; 
            }
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                               "精细角度调整，PID角速度: %.3f (误差: %.3f)", angular_z, yaw_error_1);
        }
        
        publishVelocity(0.0, 0.0, angular_z);  
        
        // 角度调整完成后，返回检测状态重新评估
        const double very_small_angular_z = 0.01;
        if (fabs(yaw_error_1) <= angle_threshold || fabs(angular_z) < very_small_angular_z) {
            RCLCPP_INFO(this->get_logger(), "角度调整完成（角度或速度阈值），返回检测状态重新评估");
            fine_tuning_initialized = false;
            target_aligned = false;  
            use_two_stage_adjustment = false;  
            transitionToState(ChassisState::DETECT_GROOVE);
            return;
        }
    }

    // 超时检查
    if (rclcpp::Clock().now() - state_start_time_ > rclcpp::Duration::from_seconds(30.0)) {
        RCLCPP_WARN(this->get_logger(), "精细调整超时，强制完成");
        fine_tuning_initialized = false;
        target_aligned = false;  
        use_two_stage_adjustment = false;  
        stopRobot();
        transitionToState(ChassisState::COMPLETED);
    }
}

void ChassisController::handleCompletedState()
{
    // 完成状态 - 任务完成，机器人停止
    stopRobot();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "任务已完成");
}

void ChassisController::transitionToState(ChassisState new_state)
{
    if (new_state != current_state_) {
        if (verbose_logging_) {
            logStateTransition(current_state_, new_state);
        }
        
        previous_state_ = current_state_;
        current_state_ = new_state;
        state_start_time_ = rclcpp::Clock().now();
        
        // 状态进入时的初始化操作
        switch (new_state) {
            case ChassisState::STOP:
            case ChassisState::DETECT_GROOVE: {
                // 重置凹槽信息，确保每次都用最新的数据重新计算
                groove_info_.detected = false;
                groove_features_buffer_.clear();
                // 清空点云数据，强制等待新的点云数据到达
                if (current_pcl_cloud_) {
                    current_pcl_cloud_->clear();
                }
                break;
            }
            case ChassisState::FINE_TUNING: {
                pid_fine_angular_z_->reset(); // 使用control_toolbox::Pid
                RCLCPP_INFO(this->get_logger(), "精细调整角速度PID控制器状态已重置（包括积分项）");
                stopRobot();
                break;
            }
            case ChassisState::COMPLETED: {
                stopRobot();
                break;
            }
            case ChassisState::MOVING: {
                // 重置里程计跟踪相关参数
                start_pose_recorded_ = false;
                start_x_ = 0.0;
                start_y_ = 0.0;
                start_yaw_ = 0.0;
                path_tracking_started_ = false;
                target_point_set_ = false;
                // 重置PID控制器
                pid_linear_x_->reset();
                pid_angular_z_->reset(); 
                RCLCPP_INFO(this->get_logger(), "直线移动PID控制器已重置");
                break;
            }
            default:
                break;
        }

        // 如果当前有活动的路径跟踪定时器，取消它
        if (path_tracking_timer_) {
            path_tracking_timer_->cancel();
            path_tracking_timer_.reset();  // 重置智能指针
        }
    }
}

void ChassisController::publishVelocity(double linear_x, double linear_y, double angular_z)
{
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear_x;
    cmd_vel.linear.y = linear_y;
    cmd_vel.angular.z = angular_z;

    // 存储上一次的速度指令
    last_commanded_velocity_ = cmd_vel;
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                       "发布速度命令: linear_x=%.4f, linear_y=%.4f, angular_z=%.4f",
                       linear_x, linear_y, angular_z);
    
    cmd_vel_pub_->publish(cmd_vel);
}

void ChassisController::stopRobot()
{
    publishVelocity(0.0, 0.0, 0.0);
}


// 回调函数实现
void ChassisController::groovePointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 检查点云是否为空
    if (msg->data.empty()) {
        return;
    }
    
    current_pointcloud_time_ = rclcpp::Clock().now();
    last_pointcloud_time_ = current_pointcloud_time_;
    pcl::fromROSMsg(*msg, *current_pcl_cloud_);
    
}

// V形凹槽特征计算函数实现
void ChassisController::calculateVGrooveFeatures()
{
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
    
    double avg_x = 0.0;
    double avg_y = 0.0;
    
    for (const auto& point : current_pcl_cloud_->points) {
        avg_x += point.x;
        avg_y += point.y;
    }
    
    avg_x /= current_pcl_cloud_->points.size();
    avg_y /= current_pcl_cloud_->points.size();
    

    double bisector_x = avg_x - center_x;
    double bisector_y = avg_y - center_y;
    // 如果向量太小，使用默认方向
    double bisector_length = sqrt(bisector_x*bisector_x + bisector_y*bisector_y);
    if (bisector_length < 0.001) {
        // 默认指向X轴正方向
        bisector_x = 1.0;
        bisector_y = 0.0;
        bisector_length = 1.0;
    }
    

    double normal_x = bisector_x / bisector_length;
    double normal_y = bisector_y / bisector_length;
    
 
    double forward_distance = 0.5;
    double forward_x = center_x + normal_x * forward_distance;
    double forward_y = center_y + normal_y * forward_distance;
    

    GrooveFeatures current_features;
    current_features.center_x = center_x;
    current_features.center_y = center_y;
    current_features.normal_x = normal_x;
    current_features.normal_y = normal_y;
    current_features.forward_x = forward_x;
    current_features.forward_y = forward_y;
    
    groove_features_buffer_.push_back(current_features);
    
    // 保持缓存大小
    if (groove_features_buffer_.size() > MAX_GROOVE_FEATURES_BUFFER_SIZE) {
        groove_features_buffer_.pop_front();
    }
    
    RCLCPP_INFO(this->get_logger(),  "当前帧特征: 中心点(%.3f, %.3f), 法向量(%.3f, %.3f), 前向点(%.3f, %.3f) [缓存:%zu帧]",
               center_x, center_y, normal_x, normal_y, forward_x, forward_y, groove_features_buffer_.size());
    
    if (groove_features_buffer_.size() >= MAX_GROOVE_FEATURES_BUFFER_SIZE) {
        updateAverageGrooveInfo();
    }
}

void ChassisController::updateAverageGrooveInfo()
{
    if (groove_features_buffer_.empty()) {
        groove_info_.detected = false;
        return;
    }
    
    // 计算所有特征的平均值
    double avg_center_x = 0.0, avg_center_y = 0.0;
    double avg_normal_x = 0.0, avg_normal_y = 0.0;
    double avg_forward_x = 0.0, avg_forward_y = 0.0;
    
    for (const auto& features : groove_features_buffer_) {
        avg_center_x += features.center_x;
        avg_center_y += features.center_y;
        avg_normal_x += features.normal_x;
        avg_normal_y += features.normal_y;
        avg_forward_x += features.forward_x;
        avg_forward_y += features.forward_y;
    }
    
    size_t buffer_size = groove_features_buffer_.size();
    avg_center_x /= buffer_size;
    avg_center_y /= buffer_size;
    avg_normal_x /= buffer_size;
    avg_normal_y /= buffer_size;
    avg_forward_x /= buffer_size;
    avg_forward_y /= buffer_size;
    
    // 归一化法向量
    double normal_length = sqrt(avg_normal_x*avg_normal_x + avg_normal_y*avg_normal_y);
    if (normal_length > 0.001) {
        avg_normal_x /= normal_length;
        avg_normal_y /= normal_length;
    }
    
    // 更新结构体
    groove_info_.detected = true;
    groove_info_.center_x = avg_center_x;
    groove_info_.center_y = avg_center_y;
    groove_info_.normal_x = avg_normal_x;
    groove_info_.normal_y = avg_normal_y;
    groove_info_.forward_x = avg_forward_x;
    groove_info_.forward_y = avg_forward_y;
    
    RCLCPP_INFO(this->get_logger(), "平均特征计算完成 [%zu帧]:", buffer_size);
    RCLCPP_INFO(this->get_logger(), "  平均中心点: (%.3f, %.3f)", avg_center_x, avg_center_y);
    RCLCPP_INFO(this->get_logger(), "  平均法向量: (%.3f, %.3f)", avg_normal_x, avg_normal_y);
    RCLCPP_INFO(this->get_logger(), "  平均前向点: (%.3f, %.3f)", avg_forward_x, avg_forward_y);
}

// 工具函数实现
std::string ChassisController::stateToString(ChassisState state)
{
    switch (state) {
        case ChassisState::STOP: return "STOP";
        case ChassisState::DETECT_GROOVE: return "DETECT_GROOVE";
        case ChassisState::MOVING: return "MOVING";
        case ChassisState::FINE_TUNING: return "FINE_TUNING";
        case ChassisState::COMPLETED: return "COMPLETED";
        default: return "UNKNOWN";
    }
}

void ChassisController::logStateTransition(ChassisState from, ChassisState to)
{
    RCLCPP_INFO(this->get_logger(), "状态转换: %s -> %s", 
               stateToString(from).c_str(), stateToString(to).c_str());
}

void ChassisController::transformLidarToBody(double lidar_x, double lidar_y, double& body_x, double& body_y)
{

    body_x = lidar_x;
    body_y = lidar_y;
    
}

// 获取当前机器人位姿的函数
bool ChassisController::getCurrentPose(double& x, double& y, double& yaw)
{
    try {
        // 获取从odom到base_link的变换
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped = tf_buffer_->lookupTransform(
            odom_frame_, base_frame_, tf2::TimePointZero);
        
        x = transform_stamped.transform.translation.x;
        y = transform_stamped.transform.translation.y;

        tf2::Quaternion quat;
        tf2::fromMsg(transform_stamped.transform.rotation, quat);
        double roll, pitch;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        
        return true;
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                             "无法获取变换 %s -> %s: %s", 
                             odom_frame_.c_str(), base_frame_.c_str(), ex.what());
        return false;
    }
}

void ChassisController::startDirectMovement()
{
    if (!target_point_set_) {
        RCLCPP_WARN(this->get_logger(), "目标点未设置，无法开始直线移动");
        stopRobot();
        return;
    }
    
    // 获取当前机器人位置
    double current_x, current_y, current_yaw;
    if (!getCurrentPose(current_x, current_y, current_yaw)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "无法获取当前位置，暂停移动");
        stopRobot();
        transitionToState(ChassisState::DETECT_GROOVE);
        return;
    }
    
    // 记录起始位置（仅在首次进入时）
    if (!start_pose_recorded_) {
        start_x_ = current_x;
        start_y_ = current_y;
        start_yaw_ = current_yaw;
        start_pose_recorded_ = true;
        RCLCPP_INFO(this->get_logger(), "记录直线移动起始位置: (%.3f, %.3f, %.3f)", start_x_, start_y_, start_yaw_);
    }
    
    const double control_frequency = 20.0;  // 20Hz控制频率
    auto timer_period = std::chrono::duration<double>(1.0 / control_frequency);
    
    // 记录初始时间
    path_tracking_start_time_ = std::chrono::steady_clock::now();
    path_tracking_last_log_time_ = path_tracking_start_time_;
    
    // 创建并启动定时器
    path_tracking_timer_ = this->create_wall_timer(
        timer_period, [this]() { this->pathTrackingCallback(); });
    
    RCLCPP_INFO(this->get_logger(), "启动直线移动控制，控制频率: %.1f Hz", control_frequency);
}


void ChassisController::pathTrackingCallback()
{
    if (!rclcpp::ok()) {
        RCLCPP_WARN(this->get_logger(), "直线移动回调收到停止信号");
        stopRobot();
        if(path_tracking_timer_) path_tracking_timer_->cancel();
        return;
    }

    // 检查是否需要退出循环（凹槽丢失或状态改变）
    if (current_state_ != ChassisState::MOVING || !groove_info_.detected) {
        RCLCPP_INFO(this->get_logger(), "退出直线移动：状态=%s，凹槽检测=%s",
                   stateToString(current_state_).c_str(),
                   groove_info_.detected ? "true" : "false");
        stopRobot();
        if(path_tracking_timer_) path_tracking_timer_->cancel();
        return;
    }
        
    // 获取当前机器人位置
    double current_x, current_y, current_yaw;
    if (!getCurrentPose(current_x, current_y, current_yaw)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "无法获取当前位置，暂停移动");
        stopRobot();
        if(path_tracking_timer_) path_tracking_timer_->cancel();
        transitionToState(ChassisState::DETECT_GROOVE);
        return;
    }
        
    // 计算相对于起始位置的偏移
    double rel_x = current_x - start_x_;
    double rel_y = current_y - start_y_;
        
    // 将里程计坐标转换为路径坐标系（考虑起始航向角）
    double cos_yaw = cos(-start_yaw_);
    double sin_yaw = sin(-start_yaw_);
    double path_x = rel_x * cos_yaw - rel_y * sin_yaw;
    double path_y = rel_x * sin_yaw + rel_y * cos_yaw;
    
    // 当前机器人位置（在路径坐标系中）
    double current_pos_x = path_x;
    double current_pos_y = path_y;
    double current_yaw_1 = current_yaw - start_yaw_;
    // 计算到目标点的距离
    double dx = target_point_.x - current_pos_x;
    double dy = target_point_.y - current_pos_y;
    double distance_to_target = sqrt(dx*dx + dy*dy);
    
    if (distance_to_target < 0.12) {  
        RCLCPP_INFO(this->get_logger(), "到达目标点，粗略移动完成！当前位置: (%.3f, %.3f), 距离目标点: %.3f米", 
                   current_pos_x, current_pos_y, distance_to_target);
        stopRobot();
        coarse_movement_completed_ = true;
        path_tracking_timer_->cancel();
        transitionToState(ChassisState::DETECT_GROOVE);
        return;
    }
    
    const double max_target_speed = 0.2; 
    const double min_target_speed = 0.05; 
    double target_speed = max_target_speed;
    if (distance_to_target < 0.3) {
        target_speed = min_target_speed + (max_target_speed - min_target_speed) * (distance_to_target / 0.3);
    }

    double target_angle = atan2(target_point_.y, target_point_.x);
    
    // 计算机器人需要转向的角度
    double yaw_error = target_angle - current_yaw_1;
    
    // 处理角度跳跃问题
    if (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    if (yaw_error < -M_PI) yaw_error += 2 * M_PI;
    
    // 根据距离目标点的远近调整角速度控制策略
    double angular_z = 0.0;
    
    if (distance_to_target > 0.3) {
        // 距离目标较远时，使用PID控制
        rclcpp::Duration dt = rclcpp::Duration::from_nanoseconds(50000000); // 50ms (20Hz)
        angular_z = pid_angular_z_->computeCommand(yaw_error, dt);
        
        // 限制角速度，避免过度转向
        const double max_angular_speed = 0.8;
        if (fabs(angular_z) > max_angular_speed) {
            angular_z = (angular_z > 0) ? max_angular_speed : -max_angular_speed;
        }
    } else {
        // 距离目标较近时，使用更保守的角速度控制
        const double max_angular_speed_near = 0.4;
        angular_z = std::min(std::max(yaw_error * 0.8, -max_angular_speed_near), max_angular_speed_near);
    }

    // 3. 发布速度指令 (linear.y始终为0)
    publishVelocity(target_speed, 0.0, angular_z);
    
    // 定期打印调试信息（每2秒打印一次）
    auto current_time = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(current_time - path_tracking_last_log_time_).count() >= 2) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                           "直线移动[20Hz]: v=%.2f, w=%.2f, 距离目标=%.2f, 角度误差=%.3f, 目标角度=%.3f, 当前朝向=%.3f",
                           target_speed, angular_z, distance_to_target, yaw_error, target_angle, current_yaw);
        path_tracking_last_log_time_ = current_time;
    }
}

// 统一PID控制器初始化函数
void ChassisController::initializePIDControllers()
{
    // 获取当前节点的shared_ptr
    auto node_ptr = rclcpp::Node::shared_from_this();
    
    // 第一部分：常规移动控制PID参数声明
    this->declare_parameter("linear_x_pid.p", 1.0);
    this->declare_parameter("linear_x_pid.i", 0.005);
    this->declare_parameter("linear_x_pid.d", 0.15);
    this->declare_parameter("linear_x_pid.i_max", 0.3);
    this->declare_parameter("linear_x_pid.i_min", -0.15);
    
    this->declare_parameter("angular_z_pid.p", 1.5);
    this->declare_parameter("angular_z_pid.i", 0.005);
    this->declare_parameter("angular_z_pid.d", 0.3);
    this->declare_parameter("angular_z_pid.i_max", 0.25);
    this->declare_parameter("angular_z_pid.i_min", -0.25);
    this->declare_parameter("fine_tuning/linear_x_pid.p", 0.5);
    this->declare_parameter("fine_tuning/linear_x_pid.i", 0.01);
    this->declare_parameter("fine_tuning/linear_x_pid.d", 0.15);
    this->declare_parameter("fine_tuning/linear_x_pid.i_max", 0.1);
    this->declare_parameter("fine_tuning/linear_x_pid.i_min", -0.1);
    
    this->declare_parameter("fine_tuning/angular_z_pid.p", 0.3);   
    this->declare_parameter("fine_tuning/angular_z_pid.i", 0.001); 
    this->declare_parameter("fine_tuning/angular_z_pid.d", 0.05);  
    this->declare_parameter("fine_tuning/angular_z_pid.max",0.1);
    this->declare_parameter("fine_tuning/angular_z_pid.min",-0.1);
    // 创建所有PID控制器实例（使用相同的节点上下文）
    RCLCPP_INFO(this->get_logger(), "创建PID控制器实例...");
    
    // 常规移动控制PID
    pid_linear_x_ = std::make_shared<control_toolbox::PidROS>(node_ptr, "linear_x_pid");
    pid_angular_z_ = std::make_shared<control_toolbox::PidROS>(node_ptr, "angular_z_pid");
    
    pid_fine_angular_z_ = std::make_shared<control_toolbox::PidROS>(node_ptr, "fine_tuning/angular_z_pid");
    
    // 初始化所有PID控制器
    RCLCPP_INFO(this->get_logger(), "初始化PID控制器...");
    
    // 初始化常规移动控制PID
    bool success = pid_linear_x_->initPid();
    RCLCPP_INFO(this->get_logger(), "线速度PID初始化%s", success ? "成功" : "失败");
    
    success = pid_angular_z_->initPid();
    RCLCPP_INFO(this->get_logger(), "角速度PID初始化%s", success ? "成功" : "失败");
    
    success = pid_fine_angular_z_->initPid();
    RCLCPP_INFO(this->get_logger(), "精细调整角速度PID初始化%s", success ? "成功" : "失败");
    
    // 确保所有PID参数被正确加载
    auto gains = pid_fine_angular_z_->getGains();
    RCLCPP_INFO(this->get_logger(), "精细调整角速度PID参数: P=%.3f, I=%.3f, D=%.3f, I_max=%.3f, I_min=%.3f",
                gains.p_gain_, gains.i_gain_, gains.d_gain_, gains.i_max_, gains.i_min_);
    if(std::isnan(gains.p_gain_ ) || std::isnan(gains.i_gain_) || std::isnan(gains.d_gain_) || std::isnan(gains.i_max_) || std::isnan(gains.i_min_))
     {
        RCLCPP_WARN(this->get_logger(), "检测到精细调整角速度PID参数有NaN值，尝试手动设置");
                    
        // 从参数服务器获取参数
        double p = this->get_parameter("fine_tuning/angular_z_pid.p").as_double();
        double i = this->get_parameter("fine_tuning/angular_z_pid.i").as_double();
        double d = this->get_parameter("fine_tuning/angular_z_pid.d").as_double();
        double i_max = this->get_parameter("fine_tuning/angular_z_pid.max").as_double();
        double i_min = this->get_parameter("fine_tuning/angular_z_pid.min").as_double();
                    
        // 创建Gains对象
        control_toolbox::Pid::Gains new_gains;
        new_gains.p_gain_ = p;
        new_gains.i_gain_ = i;
        new_gains.d_gain_ = d;
        new_gains.i_max_ = i_max;
        new_gains.i_min_ = i_min;
                    
        // 设置PID参数
        pid_fine_angular_z_->setGains(new_gains);
                    
        // 验证参数设置
        auto updated_gains = pid_fine_angular_z_->getGains();
        RCLCPP_INFO(this->get_logger(), "手动设置后的精细调整角速度PID参数: P=%.3f, I=%.3f, D=%.3f, I_max=%.3f, I_min=%.3f",
                    updated_gains.p_gain_, updated_gains.i_gain_, updated_gains.d_gain_, 
                    updated_gains.i_max_, updated_gains.i_min_);
    }
    RCLCPP_INFO(this->get_logger(), "所有PID控制器初始化完成");
}

void ChassisController::initializeFineTuningPIDControllers()
{
    // 此函数保留但为空，所有初始化已在initializePIDControllers中完成
    RCLCPP_INFO(this->get_logger(), "精细调整PID控制器已在主初始化函数中初始化");
}

// 服务回调函数实现
void ChassisController::startTaskCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                        std_srvs::srv::Trigger::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "收到启动任务服务请求");
    
    std::lock_guard<std::mutex> lock(node_state_mutex_);
    
    if (node_state_ == NodeState::ACTIVE) {
        response->success = false;
        response->message = "节点已经处于激活状态";
        RCLCPP_WARN(this->get_logger(), "节点已经处于激活状态，无法重复启动");
        return;
    }
    
    // 直接在这里激活节点，避免重复锁定
    // 创建控制定时器
    auto timer_period = std::chrono::duration<double>(1.0 / control_frequency_);
    control_timer_ = this->create_wall_timer(
        timer_period, [this]() { this->run(); });
    
    node_state_ = NodeState::ACTIVE;
    
    // 重置任务状态
    task_started_ = false;
    current_state_ = ChassisState::STOP;
    previous_state_ = ChassisState::STOP;
    coarse_movement_completed_ = false;
    
    response->success = true;
    response->message = "节点已成功激活，开始执行任务";
    RCLCPP_INFO(this->get_logger(), "节点已激活，控制定时器已启动");
}

void ChassisController::sleepNodeCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                        std_srvs::srv::Trigger::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "收到休眠节点服务请求");
    
    std::lock_guard<std::mutex> lock(node_state_mutex_);
    
    if (node_state_ == NodeState::SLEEP) {
        response->success = false;
        response->message = "节点已经处于休眠状态";
        RCLCPP_WARN(this->get_logger(), "节点已经处于休眠状态，无法重复休眠");
        return;
    }
    
    // 直接在这里休眠节点，避免重复锁定
    // 停止机器人
    stopRobot();
    
    // 取消所有定时器
    if (control_timer_) {
        control_timer_->cancel();
        control_timer_.reset();
    }
    
    if (path_tracking_timer_) {
        path_tracking_timer_->cancel();
        path_tracking_timer_.reset();
    }
    
    // 重置状态
    current_state_ = ChassisState::STOP;
    previous_state_ = ChassisState::STOP;
    task_started_ = false;
    coarse_movement_completed_ = false;
    
    node_state_ = NodeState::SLEEP;
    
    response->success = true;
    response->message = "节点已成功休眠";
    RCLCPP_INFO(this->get_logger(), "节点已休眠，所有定时器已取消");
}

void ChassisController::getStatusCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                        std_srvs::srv::Trigger::Response::SharedPtr response)
{
    std::lock_guard<std::mutex> lock(node_state_mutex_);
    
    // 检查节点是否激活
    if (node_state_ != NodeState::ACTIVE) {
        response->success = false;
        response->message = "节点处于休眠状态";
        return;
    }
    
    // 检查任务是否完成
    if (current_state_ == ChassisState::COMPLETED) {
        response->success = true;
        response->message = "对齐任务已完成";
        RCLCPP_INFO(this->get_logger(), "状态查询：对齐任务已完成");
    } else {
        response->success = false;
        response->message = "对齐任务进行中，当前状态: " + stateToString(current_state_);
        RCLCPP_INFO(this->get_logger(), "状态查询：对齐任务进行中，状态: %s", stateToString(current_state_).c_str());
    }
}

bool ChassisController::isNodeActive()
{
    std::lock_guard<std::mutex> lock(node_state_mutex_);
    return node_state_ == NodeState::ACTIVE;
}

// 添加新的初始化方法
void ChassisController::initialize()
{
    if (is_initialized_) {
        return;
    }
    
    // 初始化TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // 创建发布者
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    groove_vis_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("groove_visualization", 10);
    
    groove_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "groove_points", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            this->groovePointsCallback(msg);
        });
    
    // 创建服务
    start_task_service_ = this->create_service<std_srvs::srv::Trigger>(
        "start_task",
        std::bind(&ChassisController::startTaskCallback, this, 
                  std::placeholders::_1, std::placeholders::_2));
    
    sleep_node_service_ = this->create_service<std_srvs::srv::Trigger>(
        "sleep_node",
        std::bind(&ChassisController::sleepNodeCallback, this, 
                  std::placeholders::_1, std::placeholders::_2));
    
    get_status_service_ = this->create_service<std_srvs::srv::Trigger>(
        "get_status",
        std::bind(&ChassisController::getStatusCallback, this, 
                  std::placeholders::_1, std::placeholders::_2));
    
    // 初始化时间相关变量
    state_start_time_ = rclcpp::Clock().now();
    last_pointcloud_time_ = rclcpp::Time(0);  // 初始化为0时间戳
    
    initializePIDControllers();
    
    is_initialized_ = true;
    
    RCLCPP_INFO(this->get_logger(), "底盘控制器初始化完成 - 节点处于休眠状态");
    RCLCPP_INFO(this->get_logger(), "使用服务 /start_task 启动任务");
    RCLCPP_INFO(this->get_logger(), "使用服务 /sleep_node 休眠节点");
}

// 主函数
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // 直接创建节点指针
    auto node = std::make_shared<ChassisController>();
    
    // 初始化节点
    node->initialize();
    
    std::cout << "底盘控制器初始化完成 - 节点处于休眠状态" << std::endl;
    std::cout << "使用服务 /start_task 启动任务" << std::endl;
    std::cout << "使用服务 /sleep_node 休眠节点" << std::endl;
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "节点运行异常: " << e.what() << std::endl;
    }
    
    rclcpp::shutdown();
    return 0;
} 