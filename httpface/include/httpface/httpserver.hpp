#pragma once
#include <rclcpp/rclcpp.hpp>
#include "httplib.h"
#include <atomic>
#include <thread>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mutex>
#include <future>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nlohmann/json.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <web_control_msgs/action/map_save.hpp>
#include <web_control_msgs/srv/run_control.hpp>
#include <filesystem>
#include <regex>

class HttpServer : public rclcpp::Node{
public:
 
    explicit HttpServer(int port);

    ~HttpServer();
    
    void start(int port);

    void stop();

    bool is_running() const;

private:
    std::unique_ptr<httplib::Server> svr_;
    std::atomic<bool> running_{false};
    std::thread server_thread_;
    void setup_routes();
    int port_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr stop_timer_;
    double linear_ = 0.5;
    double angular_ = 0.5;
    int duration_ = 500;
    std::mutex cmd_mutex_;

    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr goal_status_sub_;
    std::mutex status_mutex_;
    // std::string current_status_ = R"({"status":-1})";
    nlohmann::json current_status_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr toPose_client_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    std::mutex map_mutex_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    std::mutex path_mutex_;
    nlohmann::json path_json_;

    rclcpp_action::Client<web_control_msgs::action::MapSave>::SharedPtr map_client_;
    std::mutex map_save_mutex_;
    int map_save_status_ = -1;

    std::string nav2_bringup_dir_;

    rclcpp::Client<web_control_msgs::srv::RunControl>::SharedPtr run_client_; 
    
    void NavstatusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg){
        std::lock_guard<std::mutex> lock(status_mutex_);
        if(!msg->status_list.empty()){
            const auto& latest = msg->status_list.back();
            current_status_ = {
                {"goal_id", uuidToStr(latest.goal_info.goal_id.uuid)},
                {"status:", 1},
                {"data", latest.status},
                {"message", "ok"}
            };
        }
        else{
            current_status_ = {
                {"status:", 0},
                {"message", "Failed to get nav status."}
            };
        } 
    }

    std::string uuidToStr(const std::array<uint8_t, 16>& uuid) {
        std::stringstream ss;
        for (auto byte : uuid) {
          ss << std::hex << std::setw(2) << std::setfill('0') << (int)byte;
        }
        return ss.str();
    }

    bool getRobotPose(geometry_msgs::msg::PoseStamped & pose){
        try{
            auto now = this->get_clock()->now();
            geometry_msgs::msg::TransformStamped transformStamped = 
                tf_buffer_->lookupTransform("map", "base_link", now,
                                            rclcpp::Duration::from_seconds(0.1));
            pose.header = transformStamped.header;
            pose.pose.position.x = transformStamped.transform.translation.x;
            pose.pose.position.y = transformStamped.transform.translation.y;
            pose.pose.position.z = transformStamped.transform.translation.z;
            pose.pose.orientation = transformStamped.transform.rotation;
            return true;
        }catch(tf2::TransformException & ex){
            RCLCPP_WARN(this->get_logger(), "Could not transform map to base_link: %s", ex.what());
            return false;
        }
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        std::lock_guard<std::mutex> lock(map_mutex_);
        latest_map_ = msg; 
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg){
        std::lock_guard<std::mutex> lock(path_mutex_);
        // nlohmann::json path_json;
        path_json_.clear();
        for(const auto& pose : msg->poses){
            path_json_["x"].push_back(pose.pose.position.x);
            path_json_["y"].push_back(pose.pose.position.y);
            path_json_["z"].push_back(pose.pose.position.z);
            /* path_json["poses"].push_back({
                {"x", pose.pose.position.x},
                {"y", pose.pose.position.y},
                {"z", pose.pose.position.z}
            });  */
        }
        path_json_["status"] = 1;
        path_json_["message"] = "ok";

        // path_json_ = path_json;
    }
};

