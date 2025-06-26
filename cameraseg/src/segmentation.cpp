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
            std::bind(&LowObsDetector::pointCallback1, this, std::placeholders::_1)
        ); 
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_topic_, 10);
        
    }

private:
    void pointCallback1(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud_raw);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_raw);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.05, 0.8);
        pass.filter(*cloud_filtered);

        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud_filtered);
        voxel.setLeafSize(0.008f, 0.008f, 0.008f);
        voxel.filter(*cloud_filtered);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_filtered);
        sor.setMeanK(20);
        sor.setStddevMulThresh(0.7);
        sor.filter(*cloud_filtered);

        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud_filtered);
        ne.setSearchMethod(tree);
        ne.setKSearch(20);
        ne.compute(*normals);

        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize(10);
        reg.setMaxClusterSize(100000);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(10);
        reg.setInputCloud(cloud_filtered);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(5.0 / 180.0 * M_PI);
        reg.setCurvatureThreshold(0.5);

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

            float dot = avg_normal.dot(Eigen::Vector3f(0.0f, 0.906f, 0.423f));
            dot = std::min(std::max(dot, -1.0f), 1.0f);
            // double angle_rad = std::acos(dot);

            dot = std::abs(dot);

            /* if((angle_rad > 1.047 && angle_rad < 1.221) ||
               (angle_rad > 1.92  && angle_rad < 2.09)){ */
            
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
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}