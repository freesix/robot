#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <std_msgs/msg/color_rgba.hpp>


class LowObsDetector : public rclcpp::Node{
public:
    LowObsDetector() : Node("low_obs"){
        this->declare_parameter("topic", "/camera/depth/color/points");
        this->declare_parameter("pub", "/low_obs_left");
        sub_topic_ = this->get_parameter("topic").as_string();
        pub_topic_ = this->get_parameter("pub").as_string();

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            sub_topic_, rclcpp::SensorDataQoS(),
            std::bind(&LowObsDetector::pointCallback, this, std::placeholders::_1)
        ); 
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_topic_, 1);
        
    }

private:

    void pointCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud_raw);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        // Eigen::Vector3f target_normal(0.0f, -0.906f, -0.423f);
        Eigen::Vector3f target_normal(0.0f, -0.9397f, 0.3420f);
        // extractPlanesWithNormal(cloud_raw, cloud_filtered, target_normal);
        removePlanesWithNormal(cloud_raw, cloud_filtered, target_normal);
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_filtered, output_msg);
        output_msg.header = msg->header;
        pub_->publish(output_msg);
    
    }

    void removePlanesWithNormal(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
    const Eigen::Vector3f& target_normal,
    float angle_thresh_deg = 20.0,
    int max_planes = 5)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remaining(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_in);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        sor.filter(*cloud_remaining);
/*         int planes_removed = 0;

        while (planes_removed < max_planes && cloud_remaining->size() > 50) {
            // 1. RANSAC 平面分割
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.01);
            // seg.setMaxIterations(2000);
            // seg.setProbability(0.999); 
            seg.setInputCloud(cloud_remaining);

            pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            seg.segment(*inliers, *coeffs);

            if (inliers->indices.empty()) {
                break;
            }

            // 2. 判断平面法向量是否接近目标
            Eigen::Vector3f plane_normal(coeffs->values[0], coeffs->values[1], coeffs->values[2]);
            // RCLCPP_INFO_STREAM(this->get_logger(), "noraml: "<<coeffs->values[0]<<","<<
                // coeffs->values[1]<<","<<coeffs->values[2]);
            float dot = plane_normal.dot(target_normal);
            float angle = std::acos(dot / (plane_normal.norm() * target_normal.norm()));

            if (angle < angle_thresh_deg * M_PI / 180.0) {
                // 3. 剔除这个平面
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(cloud_remaining);
                extract.setIndices(inliers);
                extract.setNegative(true);
                pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
                extract.filter(*tmp);
                cloud_remaining = tmp;

                ++planes_removed;
                // RCLCPP_INFO_STREAM(this->get_logger(), "移除方向匹配平面 " << planes_removed << "，剩余点数：" << cloud_remaining->size());
            } else {
                // 平面方向不匹配，跳出（不移除）
                break;
            }
        }
 */
        *cloud_out = *cloud_remaining;

        // RCLCPP_INFO_STREAM(this->get_logger(), "完成：移除了 " << planes_removed << " 个方向匹配的平面。");
    }

    void extractPlanesWithNormal(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
        pcl::PointCloud<pcl::PointXYZ>::Ptr matched_planes_out,
        const Eigen::Vector3f& target_normal,
        float angle_thresh_deg = 10.0,
        int max_planes = 5)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remaining(new pcl::PointCloud<pcl::PointXYZ>(*cloud_in));
        matched_planes_out->clear();

        int planes_found = 0;

        while (planes_found < max_planes && cloud_remaining->size() > 50) {
            // 1. 平面分割
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.01);
            seg.setInputCloud(cloud_remaining);

            pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            seg.segment(*inliers, *coeffs);

            if (inliers->indices.empty()) {
                break; // 无平面
            }

            // 2. 判断法向量是否接近目标
            Eigen::Vector3f plane_normal(coeffs->values[0], coeffs->values[1], coeffs->values[2]);
            float dot = plane_normal.dot(target_normal);
            float angle = std::acos(dot / (plane_normal.norm() * target_normal.norm()));

            if (angle < angle_thresh_deg * M_PI / 180.0) {
                // 3. 法向量接近目标，提取内点
                pcl::PointCloud<pcl::PointXYZ> plane_points;
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(cloud_remaining);
                extract.setIndices(inliers);
                extract.setNegative(false);
                extract.filter(plane_points);

                // 合并到输出结果
                *matched_planes_out += plane_points;

                RCLCPP_INFO_STREAM(this->get_logger(), "匹配平面 " << planes_found + 1 << "，提取点数：" << plane_points.size());
                ++planes_found;
            }

            // 4. 移除当前平面点，进行下一轮
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud_remaining);
            extract.setIndices(inliers);
            extract.setNegative(true);
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
            extract.filter(*tmp);
            cloud_remaining = tmp;
        }

        if (planes_found == 0) {
            RCLCPP_INFO(this->get_logger(), "未发现方向接近目标的平面。");
        } else {
            RCLCPP_INFO_STREAM(this->get_logger(), "共找到 " << planes_found << " 个方向匹配的平面，点数：" << matched_planes_out->size());
        }
    }

    void pointCallback1(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud_raw);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_raw);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-0.1, 0.68);
        pass.filter(*cloud_filtered);

        // pass.setInputCloud(cloud_filtered);
        // pass.setFilterFieldName("y");
        // pass.setFilterLimits(-0.1, 0.68);
        // pass.filter(*cloud_filtered);

        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud_filtered);
        voxel.setLeafSize(0.01f, 0.01f, 0.01f);
        voxel.filter(*cloud_filtered);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_filtered);
        sor.setMeanK(30);
        sor.setStddevMulThresh(0.7);
        sor.filter(*cloud_filtered);

        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud_filtered);
        ne.setSearchMethod(tree);
        ne.setKSearch(30);
        ne.compute(*normals);

        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize(20);
        reg.setMaxClusterSize(5000);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(20);
        reg.setInputCloud(cloud_filtered);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(4.0 / 180.0 * M_PI);
        reg.setCurvatureThreshold(0.2);

        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters);

        Eigen::Vector3f avg_normal(0, 0, 0);
        for (size_t i = 0; i < clusters.size(); ++i) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (int idx : clusters[i].indices) {
                const pcl::Normal &normal = normals->points[idx];
                avg_normal += Eigen::Vector3f(normal.normal_x, normal.normal_y, normal.normal_z);
            }
            avg_normal.normalize();

            float dot = avg_normal.dot(Eigen::Vector3f(0.0f, -0.9397f, 0.3420f));
            dot = std::min(std::max(dot, -1.0f), 1.0f);
            // double angle_rad = std::acos(dot);

            dot = std::abs(dot);

            //  if((angle_rad > 1.047 && angle_rad < 1.221) ||
            //    (angle_rad > 1.92  && angle_rad < 2.09)){
            
            if(dot > 0.98){
            
                clusters.erase(clusters.begin() + i);
                // break;
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr low_obstacles(new pcl::PointCloud<pcl::PointXYZ>);
        for(const auto &indices : clusters){
            for(int idx : indices.indices){
                low_obstacles->points.push_back(cloud_filtered->points[idx]);
            }
        }

 
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*low_obstacles, output_msg);
        // pcl::toROSMsg(*cloud_filtered, output_msg);
        output_msg.header = msg->header;
        pub_->publish(output_msg);
    
    }


    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    std::string sub_topic_;
    std::string pub_topic_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LowObsDetector>();
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}