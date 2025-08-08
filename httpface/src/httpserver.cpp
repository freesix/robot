#include "httpface/httpserver.hpp"

using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

HttpServer::HttpServer(int port) : Node("http_server"), port_(port){
    svr_ = std::make_unique<httplib::Server>();
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    goal_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
        "navigate_to_pose/_action/status", rclcpp::SystemDefaultsQoS(), 
        std::bind(&HttpServer::NavstatusCallback, this, std::placeholders::_1)); 
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    toPose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&HttpServer::mapCallback, this, std::placeholders::_1));
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/plan", rclcpp::SensorDataQoS(), std::bind(&HttpServer::pathCallback, this, std::placeholders::_1));

    start(port_);
};

HttpServer::~HttpServer(){
    if(server_thread_.joinable()){
        server_thread_.join();
    }
}

void HttpServer::start(int port){
    if(running_){
        RCLCPP_ERROR(this->get_logger(), "Http Server is running.");
        return;
    }

    setup_routes();

    server_thread_ = std::thread([this, port](){
        running_ = true;
        RCLCPP_INFO_STREAM(this->get_logger(), "Http Server starting on port "<<port);
        svr_->listen("0.0.0.0", port);
        running_ = false;
        RCLCPP_INFO(this->get_logger(), "Http Server stopped.");
    });
}

void HttpServer::stop(){
    if(!running_){
        RCLCPP_ERROR(this->get_logger(), "Http Server is not running.");
        return;
    }
    httplib::Client cli("localhost", port_);
    auto res = cli.Get("/stop");

    if(server_thread_.joinable()){
        server_thread_.join();
    }
}


bool HttpServer::is_running() const{
    return running_;
}

void HttpServer::setup_routes(){
    svr_->Get("/stop", [&](const httplib::Request &req, httplib::Response &res){
        res.set_content("Stopping server...", "text/plain");
        svr_->stop();
    });
    svr_->Get("/hi", [](const httplib::Request& req, httplib::Response& res) {
        res.set_content("Hello World!", "text/plain");
    });
    svr_->Post("/move", [this](const httplib::Request &req, httplib::Response &res){
        try{
            auto json = nlohmann::json::parse(req.body);
            double linear = json["linear"];
            double angular = json["angular"];

            geometry_msgs::msg::Twist twist;
            twist.linear.x = linear;    // 线速度
            twist.angular.z = angular;  // 角速度

            cmd_pub_->publish(twist);
            RCLCPP_INFO(this->get_logger(), "Executed move command: linear=%.2f, angular=%.2f", linear, angular);

            nlohmann::json response_json = {{"status", "OK"}, {"linear", linear}, {"angular", angular}};
            res.set_content(response_json.dump(), "application/json"); 
        }
        catch(const std::exception &e){
            res.status = 400;
            nlohmann::json error_json = {{"error", "Invalid JSON or missing 'linear' or 'angular' field"}};
            res.set_content(error_json.dump(), "application/json");
        }
    });

    svr_->Get("/nav_status", [this](const httplib::Request&, httplib::Response& res) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        res.set_content(current_status_, "application/json");
    });
    
    svr_->Get("/robot_pose", [this](const httplib::Request&, httplib::Response& res) {
        geometry_msgs::msg::PoseStamped pose;
        bool ok = getRobotPose(pose);

        if(ok){ 
            tf2::Quaternion q(
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            std::stringstream ss;
            ss << "{"
               << "\"x\": " << pose.pose.position.x << ", "
               << "\"y\": " << pose.pose.position.y << ", "
               << "\"yaw\": " << yaw
               << "}";

            res.set_content(ss.str(), "application/json");
        }else{
            res.status = 500;
            res.set_content("{\"error\": \"Failed to get robot pose\"}", "application/json");
        }
    });
    
    svr_->Post("/send_goal", [this](const httplib::Request& req, httplib::Response& res) {
        // try{
            auto json = nlohmann::json::parse(req.body);
            double x = json["x"];
            double y = json["y"];
            double yaw = json["yaw"];
            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.header.frame_id = "map";
            target_pose.header.stamp = this->now();
            target_pose.pose.position.x = x;
            target_pose.pose.position.y = y;
            target_pose.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            target_pose.pose.orientation = tf2::toMsg(q);

            nav2_msgs::action::NavigateToPose::Goal goal;
            goal.pose = target_pose;

            if(!toPose_client_->wait_for_action_server(std::chrono::seconds(2))){
                res.status = 503;
                res.set_content("{\"error\":\"Server not available\"}", "application/json");
                return;
            }

            auto future_goal_handle = toPose_client_->async_send_goal(goal);
            if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle)!=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                res.status = 500;
                res.set_content("{\"error\":\"Failed to send goal\"}", "application/json");
                return;
            }

            auto goal_handle = future_goal_handle.get();
            if (!goal_handle) {
                res.status = 500;
                res.set_content("{\"error\":\"Goal rejected by server\"}", "application/json");
                return;
            }

            std::ostringstream oss;
            for (auto b : goal_handle->get_goal_id()) {
                oss << std::hex << std::setw(2) << std::setfill('0') << (int)b;
            }

            std::string uuid_str = oss.str();
            res.set_content("{\"goal_id\":\"" + uuid_str + "\"}", "application/json");
        // }catch(const std::exception &e){
            // res.status = 400;
            // res.set_content(std::string("{\"error\":\"") + e.what() + "\"}", "application/json");
        // }
    });

    svr_->Get("/map", [this](const httplib::Request &req, httplib::Response &res){
        std::lock_guard<std::mutex> lock(map_mutex_);
        if(!latest_map_){
            res.status = 503;
            res.set_content("{\"error\": \"No map received yet\"}", "application/json");
            return;
        }
        
        nlohmann::json j;
        const auto &msg = *latest_map_;

        j["header"]["frame_id"] = msg.header.frame_id;
        j["info"]["resolution"] = msg.info.resolution;
        j["info"]["width"] = msg.info.width;
        j["info"]["height"] = msg.info.height;
        j["info"]["origin"]["position"]["x"] = msg.info.origin.position.x;
        j["info"]["origin"]["position"]["y"] = msg.info.origin.position.y;
        j["info"]["origin"]["position"]["z"] = msg.info.origin.position.z;
        j["info"]["origin"]["orientation"]["x"] = msg.info.origin.orientation.x;
        j["info"]["origin"]["orientation"]["y"] = msg.info.origin.orientation.y;
        j["info"]["origin"]["orientation"]["z"] = msg.info.origin.orientation.z;
        j["info"]["origin"]["orientation"]["w"] = msg.info.origin.orientation.w;

        j["data"] = msg.data;

        res.set_content(j.dump(), "application/json");
    });

    svr_->Get("/path", [this](const httplib::Request& req, httplib::Response& res) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        res.set_content(path_json_.dump(2), "application/json");
    });
}