#include "groove_detector.hpp"
#include <queue>
#include <ctime>  // 添加时间头文件，用于随机数种子
#include <omp.h>
#include <std_srvs/srv/trigger.hpp>

GrooveDetector::GrooveDetector()
    : Node("groove_detector"), 
      first_frame_processed_(false),
      groove_detected_(false),
      is_active_(false)  // 默认休眠状态
{
    // 创建服务服务器
    start_detection_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "start_detection", 
        std::bind(&GrooveDetector::startDetectionCallback, this, 
                  std::placeholders::_1, std::placeholders::_2));
    
    stop_detection_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "stop_detection", 
        std::bind(&GrooveDetector::stopDetectionCallback, this, 
                  std::placeholders::_1, std::placeholders::_2));

    // 创建订阅者
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/laser/data", 10, 
        std::bind(&GrooveDetector::laserScanCallback, this, std::placeholders::_1));

    // 创建发布者
    detection_info_pub_ = this->create_publisher<std_msgs::msg::String>(
        "groove_info", 10);
    groove_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "groove_points", 10);
    filtered_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "filtered_points", 10);



    // 初始化点云对象
    current_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    groove_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    previous_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    // 初始化KD树
    kdtree_ = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>);

    // 生成V形模板
    generateVTemplate();
    
    // 初始化BBS优先队列，按分数降序排列
    bbs_queue_ = std::priority_queue<BBSTarget, std::vector<BBSTarget>, 
                                    std::function<bool(const BBSTarget&, const BBSTarget&)>>(
        [](const BBSTarget& a, const BBSTarget& b) {
            return a.score < b.score;  // 分数高的优先级高
        });

    RCLCPP_INFO(this->get_logger(), "Groove detector node initialized in SLEEPING state");
}

void GrooveDetector::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    if (!scan_msg) {
        RCLCPP_WARN(this->get_logger(), "Received null laser scan message");
        return;
    }

    // 休眠状态下直接返回，不处理数据
    if (!is_active_) {
        return;
    }

    try {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        
        // 保存时间戳和header信息
        current_scan_time_ = scan_msg->header.stamp;
        current_scan_header_ = scan_msg->header;
        
        // 创建新的点云对象
        pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // 将激光数据转换为点云，处理所有点
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            // 距离过滤
            if (std::isfinite(scan_msg->ranges[i]) && 
                scan_msg->ranges[i] >= min_range_ && 
                scan_msg->ranges[i] <= max_range_) {
                
                // 强度过滤（如果启用且强度数据可用）
                bool intensity_ok = true;
                if (enable_intensity_filter_ && 
                    !scan_msg->intensities.empty() && 
                    scan_msg->intensities.size() == scan_msg->ranges.size()) {
                    
                    float intensity = scan_msg->intensities[i];
                    intensity_ok = (intensity >= min_intensity_ && intensity <= max_intensity_);
                }
                
                // 只有通过所有过滤条件的点才被添加到点云中
                if (intensity_ok) {
                    pcl::PointXYZ point;
                    float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
                    // 先计算雷达坐标系下的坐标
                    double laser_x = scan_msg->ranges[i] * std::cos(angle);
                    double laser_y = scan_msg->ranges[i] * std::sin(angle);
                    //point.x = laser_y;   // x_body = y_laser
                    //point.y = -laser_x;  // y_body = -x_laser
                    // 按照图片中的转换公式：X = -R*sin(α), Y = -R*cos(α)
                    // 注意：这里假设angle是从Y轴正方向开始测量，如果实际是从X轴开始，需要调整
                    
                    //double range = scan_msg->ranges[i];
                    //double laser_x = range * std::sin(angle);
                    //double laser_y = range * std::cos(angle);
                    point.z = 0.0;
                    point.x = -laser_y;   // x_body = y_laser
                    point.y = laser_x;  // y_body = -x_laser
                    point.z = 0.0;
                    new_cloud->push_back(point);
                }
            }
        }

        // 设置点云的时间戳
        new_cloud->header.stamp = pcl_conversions::toPCL(scan_msg->header.stamp);
        new_cloud->header.frame_id = scan_msg->header.frame_id;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        
        // 第一步：拖尾滤波（指数移动平均滤波）
        pcl::PointCloud<pcl::PointXYZ>::Ptr trailing_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        if (!previous_cloud_->empty() && previous_cloud_->size() == new_cloud->size()) {
            // 对每个点应用指数移动平均
            trailing_filtered->resize(new_cloud->size());
            for (size_t i = 0; i < new_cloud->size(); ++i) {
                trailing_filtered->points[i].x = trailing_filter_alpha_ * new_cloud->points[i].x + 
                                               (1.0 - trailing_filter_alpha_) * previous_cloud_->points[i].x;
                trailing_filtered->points[i].y = trailing_filter_alpha_ * new_cloud->points[i].y + 
                                               (1.0 - trailing_filter_alpha_) * previous_cloud_->points[i].y;
                trailing_filtered->points[i].z = trailing_filter_alpha_ * new_cloud->points[i].z + 
                                               (1.0 - trailing_filter_alpha_) * previous_cloud_->points[i].z;
            }
        } else {
            // 第一帧或点数不匹配时，直接使用当前帧
            *trailing_filtered = *new_cloud;
        }
        
        // 第二步：MLS平滑滤波
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
        mls.setInputCloud(trailing_filtered);
        mls.setPolynomialOrder(polynomial_order_);
        mls.setSearchRadius(search_radius_);
        mls.process(*cloud_filtered);
        
        // 更新前一帧点云为当前拖尾滤波后的结果
        *previous_cloud_ = *trailing_filtered;

        // 更新当前点云为最终滤波结果
        *current_cloud_ = *cloud_filtered;

        // 构建KD树用于近邻搜索
        if (!current_cloud_->empty()) {
            kdtree_->setInputCloud(current_cloud_);
        }

        new_data_available_ = true;
        first_frame_processed_ = true;

        // 打印处理状态（每隔一段时间打印一次，避免过多日志）
        static int callback_count = 0;
        if (++callback_count % 50 == 0) {
            RCLCPP_INFO(this->get_logger(), "处理状态: 点云点数=%zu", current_cloud_->size());
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in laserScanCallback: %s", e.what());
    }
}

void GrooveDetector::run()
{
    while (rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());
        
        // 休眠状态下不执行检测逻辑，减少CPU使用
        if (!is_active_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 休眠500ms
            continue;
        }
        
        // 活跃状态下的正常处理
        syncData();
        detectGroove();
        publishResults();
    }
}

bool GrooveDetector::syncData()
{
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    
    if (!new_data_available_ || current_cloud_->empty()) {
        return false;
    }

    // 创建BBS目标容器
    std::vector<BBSTarget> bbs_targets;
    int n = current_cloud_->size();
    bbs_targets.reserve(n*10);
    // 双重循环遍历点云创建结构体并保存在容器中
    #pragma omp parallel for schedule(dynamic)
    for (size_t i = 0; i < current_cloud_->size(); ++i) {
        for (double angle = 0.0; angle < 2 * M_PI; angle += M_PI / 18.0) {  // 360度以10度为固定步长
            BBSTarget target;
            target.level = 1;  // 初始层级为0
            target.cloud_index = static_cast<int>(i);
            target.rotation_angle = angle;
            target.score = 0.0;  // 初始分数为0
            
            #pragma omp critical
            {
                bbs_targets.push_back(target);
            }
        }
    }
    
    // 循环将vector中的目标提取出来，计算得分之后将其放入priority_queue队列当中
    #pragma omp parallel for schedule(dynamic)
    for (size_t i = 0; i < bbs_targets.size(); ++i) {
        const auto& target = bbs_targets[i];
        // 计算得分
        double score = calculateScore(target);
        
        // 创建新的目标对象并设置得分
        BBSTarget scored_target = target;
        scored_target.score = score;
        
        // 将计算得分后的目标放入优先队列
        #pragma omp critical
        {
            bbs_queue_.push(scored_target);
        }
    }

    // 发布滤波后的点云
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*current_cloud_, filtered_msg);
    filtered_msg.header = current_scan_header_;
    filtered_points_pub_->publish(filtered_msg);

    new_data_available_ = false;
    
    return true;
}

// V形模板生成函数 - 按照顶点角度和边长生成
void GrooveDetector::generateVTemplate()
{
    v_template_.angle = template_angle_;
    v_template_.points.clear();
    
    // 计算V形两边的角度
    // V形开口朝向x轴正方向，顶点角度为136.5度
    double half_angle = template_angle_ / 2.0;  // 68.25度
    
    // 左边和右边相对于x轴正方向的角度
    double left_angle = half_angle;      // +68.25度
    double right_angle = -half_angle;    // -68.25度
    
    // 每条边上的点数
    int points_per_edge = template_points_num_ / 2;
    
    // 生成左边线的点（从顶点开始）
    for (int i = 0; i <= points_per_edge; ++i) {
        pcl::PointXYZ point;
        double t = static_cast<double>(i) / points_per_edge;  // 0到1的参数
        double distance = template_edge_length_ * t;  // 沿边的距离
        
        point.x = distance * std::cos(left_angle);
        point.y = distance * std::sin(left_angle);
        point.z = 0.0;
        v_template_.points.push_back(point);
    }
    
    // 生成右边线的点（跳过顶点，避免重复）
    for (int i = 1; i <= points_per_edge; ++i) {
        pcl::PointXYZ point;
        double t = static_cast<double>(i) / points_per_edge;  // 0到1的参数
        double distance = template_edge_length_ * t;  // 沿边的距离
        
        point.x = distance * std::cos(right_angle);
        point.y = distance * std::sin(right_angle);
        point.z = 0.0;
        v_template_.points.push_back(point);
    }
    
    // 计算V形开口宽度（两边端点之间的距离）
    pcl::PointXYZ left_end, right_end;
    left_end.x = template_edge_length_ * std::cos(left_angle);
    left_end.y = template_edge_length_ * std::sin(left_angle);
    right_end.x = template_edge_length_ * std::cos(right_angle);
    right_end.y = template_edge_length_ * std::sin(right_angle);
    
    double opening_width = std::sqrt(
        std::pow(left_end.x - right_end.x, 2) + 
        std::pow(left_end.y - right_end.y, 2)
    );
    
    v_template_.width = opening_width;
    v_template_.depth = template_edge_length_;
    
    RCLCPP_INFO(this->get_logger(), "V形模板生成完成，包含 %zu 个点", v_template_.points.size());
    RCLCPP_INFO(this->get_logger(), "模板参数: 边长=%.2fcm, 角度=%.1f°, 开口宽度=%.2fcm", 
               template_edge_length_ * 100, template_angle_ * 180.0 / M_PI, opening_width * 100);
}

// 模板匹配函数 - 简化版本，只计算匹配分数
double GrooveDetector::matchVTemplate(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_cloud, 
                                    const std::vector<pcl::PointXYZ>& template_points)
{
    if (scan_cloud->empty() || template_points.empty()) {
        return 0.0;
    }
    
    // 直接使用传入的点云构建KD树
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(scan_cloud);
    
    // 计算匹配分数
    double total_score = 0.0;
    int matched_count = 0;
    const double match_distance_threshold = 0.01; // 1cm匹配阈值
    
    // 准备最近邻搜索的变量
    std::vector<int> nearest_indices(1);
    std::vector<float> nearest_distances(1);
    
    for (const auto& template_point : template_points) {
        // 使用KD树进行最近邻搜索
        if (kdtree.nearestKSearch(template_point, 1, nearest_indices, nearest_distances) > 0) {
            double min_distance = std::sqrt(nearest_distances[0]); // nearestKSearch返回的是距离的平方
            
            if (min_distance < match_distance_threshold) {
                total_score += 1.0 - (min_distance / match_distance_threshold);
                matched_count++;
            }
        }
    }
    
    // 归一化分数，要求至少匹配一定比例的模板点
    if (matched_count >= template_points.size() * 0.6) {  // 至少60%的模板点匹配
        return total_score / template_points.size();
    } else {
        return 0.0;
    }
}

// BBS得分计算函数
double GrooveDetector::calculateScore(const BBSTarget& target)
{
    if (current_cloud_->empty() || target.cloud_index >= static_cast<int>(current_cloud_->size())) {
        return 0.0;
    }
    
    const pcl::PointXYZ& candidate_center = current_cloud_->points[target.cloud_index];
    
    // 跳过距离太远或太近的点
    double distance_to_origin = std::sqrt(candidate_center.x * candidate_center.x + 
                                         candidate_center.y * candidate_center.y);
    if (distance_to_origin < 0.3 || distance_to_origin > 3.0) {
        return 0.0;
    }
    
    // 使用KD树提取候选点周围的近邻点作为匹配区域
    const double search_radius = 0.15; // 15cm搜索半径
    std::vector<int> neighbor_indices;
    std::vector<float> neighbor_distances;
    
    if (kdtree_->radiusSearch(candidate_center, search_radius, neighbor_indices, neighbor_distances) < 5) {
        return 0.0; // 如果近邻点太少，返回0分
    }
    
    // 提取近邻点为PCL点云格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr neighbor_points(new pcl::PointCloud<pcl::PointXYZ>);
    neighbor_points->points.reserve(neighbor_indices.size());
    for (int idx : neighbor_indices) {
        neighbor_points->points.push_back(current_cloud_->points[idx]);
    }
    neighbor_points->width = neighbor_points->points.size();
    neighbor_points->height = 1;
    neighbor_points->is_dense = true;
    
    // 生成以candidate_center为顶点的变换后模板点云
    std::vector<pcl::PointXYZ> transformed_template;
    double cos_angle = std::cos(target.rotation_angle);
    double sin_angle = std::sin(target.rotation_angle);
    
    for (const auto& template_point : v_template_.points) {
        pcl::PointXYZ transformed_point;
        // 先旋转
        transformed_point.x = template_point.x * cos_angle - template_point.y * sin_angle;
        transformed_point.y = template_point.x * sin_angle + template_point.y * cos_angle;
        transformed_point.z = 0.0;
        
        // 再平移到候选中心（作为V形顶点）
        transformed_point.x += candidate_center.x;
        transformed_point.y += candidate_center.y;
        
        transformed_template.push_back(transformed_point);
    }
    
    // 计算模板与近邻点的匹配分数
    return matchVTemplate(neighbor_points, transformed_template);
}


void GrooveDetector::detectGroove()
{
    if (current_cloud_->empty()) {
        return;
    }
    
    // BBS分支界定算法变量
    double best_score = 0.0;
    BBSTarget best_target;
    bool valid_groove = false;
    
    // 添加异常处理，避免因算法问题导致的死锁
    try {
        // BBS主循环：剪支、分支扩展等操作
        while (!bbs_queue_.empty()) {
            // top提取当前目标
            BBSTarget current_target = bbs_queue_.top();
            // pop删除优先队列
            bbs_queue_.pop();
            
            // 剪支：如果小于当前的最佳分数则continue跳过当前循环
            if (current_target.score < best_score) {
                continue;
            }
            
            // 判断如果提取的目标层级为0则记录当前分数并复制该结构体
            if (current_target.level == 0) {
                best_score = current_target.score;
                best_target = current_target;
            } else {
                // 层级不为0则进行扩支操作
                // 扩支操作：将当前的目标点云索引不变的情况下将旋转分别增加1度，扩展8个
                for (int i = -5; i <= 4; ++i) {
                    BBSTarget new_target = current_target;
                    new_target.level = current_target.level - 1; // 层级减1
                    new_target.rotation_angle += i * M_PI / 180.0; // 增加i度
                    
                    // 确保角度在[0, 2π)范围内
                    while (new_target.rotation_angle < 0) {
                        new_target.rotation_angle += 2 * M_PI;
                    }
                    while (new_target.rotation_angle >= 2 * M_PI) {
                        new_target.rotation_angle -= 2 * M_PI;
                    }
                    
                    // 计算新目标的得分
                    new_target.score = calculateScore(new_target);
                    
                    // 将新目标加入优先队列
                    bbs_queue_.push(new_target);
                }
            }
        }
        
        // 更新检测结果
        if (best_score<0.65) {
            RCLCPP_INFO(this->get_logger(),"分数不达标");
            return;
        }
        else{
            best_groove_center_ = current_cloud_->points[best_target.cloud_index];
            best_groove_angle_ = best_target.rotation_angle;
            valid_groove = true;
            RCLCPP_INFO(this->get_logger(), "V形凹槽检测成功! 中心位置: (%.3f, %.3f), 角度: %.1f°, 匹配分数: %.3f", 
                best_groove_center_.x, best_groove_center_.y, best_groove_angle_ * 180.0 / M_PI, best_score);
        }
            
        
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "BBS检测算法出错: %s", e.what());
        valid_groove = false;
    }
    
    // 更新检测结果 - 一次性完成所有更新
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        groove_detected_ = valid_groove;
        
        // 清空之前的结果
        groove_cloud_->clear();
        
        if (groove_detected_) {
            // 生成经过平移和旋转的模板点云
            double cos_angle = std::cos(best_groove_angle_);
            double sin_angle = std::sin(best_groove_angle_);
            
            for (const auto& template_point : v_template_.points) {
                pcl::PointXYZ transformed_point;
                // 先旋转
                transformed_point.x = template_point.x * cos_angle - template_point.y * sin_angle;
                transformed_point.y = template_point.x * sin_angle + template_point.y * cos_angle;
                transformed_point.z = 0.0;
                
                // 再平移到凹槽中心
                transformed_point.x += best_groove_center_.x;
                transformed_point.y += best_groove_center_.y;
                
                groove_cloud_->push_back(transformed_point);
            }
            groove_cloud_->header = current_cloud_->header;
        }
    }
}

void GrooveDetector::publishResults()
{
    std::lock_guard<std::mutex> lock(cloud_mutex_);

    if (groove_detected_) {
        // 发布检测信息
        std_msgs::msg::String msg;
        msg.data = "Groove detected";
        detection_info_pub_->publish(msg);

        // 发布凹槽点云
        if (!groove_cloud_->empty()) {
            sensor_msgs::msg::PointCloud2 groove_msg;
            groove_msg.header = current_scan_header_;
            pcl::toROSMsg(*groove_cloud_, groove_msg);
            groove_points_pub_->publish(groove_msg);
        }
    }

    // 发布当前处理的点云
    if (!current_cloud_->empty()) {
        sensor_msgs::msg::PointCloud2 current_msg;
        current_msg.header = current_scan_header_;
        pcl::toROSMsg(*current_cloud_, current_msg);
        filtered_points_pub_->publish(current_msg);
    }

}

// 服务回调函数
void GrooveDetector::startDetectionCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (!is_active_) {
        is_active_ = true;
        RCLCPP_INFO(this->get_logger(), "Detection activated - starting groove detection");
        response->success = true;
        response->message = "Detection started";
    } else {
        response->success = false;
        response->message = "Detection already active";
    }
}

void GrooveDetector::stopDetectionCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (is_active_) {
        is_active_ = false;
        
        // 清理数据，减少内存使用
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            if (!current_cloud_->empty()) {
                current_cloud_->clear();
            }
            if (!groove_cloud_->empty()) {
                groove_cloud_->clear();
            }
            if (!previous_cloud_->empty()) {
                previous_cloud_->clear();
            }
            
            // 清空BBS队列
            while (!bbs_queue_.empty()) {
                bbs_queue_.pop();
            }
            
            // 重置检测状态
            groove_detected_ = false;
            new_data_available_ = false;
            first_frame_processed_ = false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Detection deactivated - entering sleep mode");
        response->success = true;
        response->message = "Detection stopped and memory cleaned";
    } else {
        response->success = false;
        response->message = "Detection already stopped";
    }
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GrooveDetector>();
    node->run();
    rclcpp::shutdown();
    return 0;
} 