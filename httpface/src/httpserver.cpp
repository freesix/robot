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
    nav2_bringup_dir_ = ament_index_cpp::get_package_share_directory("nav2_bringup");
    map_client_ = rclcpp_action::create_client<web_control_msgs::action::MapSave>(this, "save_map");
    run_client_ = this->create_client<web_control_msgs::srv::RunControl>("run_control");

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
    svr_->Get("/stop", [&](const httplib::Request&, httplib::Response &res){
        res.set_content("Stopping server...", "text/plain");
        svr_->stop();
    });
    svr_->Get("/hi", [](const httplib::Request&, httplib::Response& res) {
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
                // res.status = 503;
                // res.set_content("{\"error\":\"Server not available\"}", "application/json");
                return;
            }

            auto future_goal_handle = toPose_client_->async_send_goal(goal);
            if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle)!=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                // res.status = 500;
                // res.set_content("{\"error\":\"Failed to send goal\"}", "application/json");
                return;
            }

            auto goal_handle = future_goal_handle.get();
            if (!goal_handle) {
                // res.status = 500;
                // res.set_content("{\"error\":\"Goal rejected by server\"}", "application/json");
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

    svr_->Get("/map", [this](const httplib::Request&, httplib::Response &res){
        std::lock_guard<std::mutex> lock(map_mutex_);
        if(!latest_map_){
            // res.status = 503;
            // res.set_content("{\"error\": \"No map received yet\"}", "application/json");
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

    svr_->Get("/path", [this](const httplib::Request&, httplib::Response& res) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        res.set_content(path_json_.dump(2), "application/json");
    });

    svr_->Post("/map_save", [this](const httplib::Request& req, httplib::Response &res){
        auto goal = web_control_msgs::action::MapSave::Goal();
        map_save_status_ = -1;
        goal.map_path = nav2_bringup_dir_ + "/maps/map1.pbstream";
        goal.map_name = nav2_bringup_dir_ + "/maps/map1";
        goal.resolution = 0.05f;
        if(!req.body.empty()){
            try{
                auto json = nlohmann::json::parse(req.body);
                if(json.contains("pbstream_path"))goal.map_path = json["pbstream_path"].get<std::string>();
                if(json.contains("map_name"))goal.map_name = json["map_name"].get<std::string>();
                if(json.contains("resolution"))goal.resolution = json["resolution"].get<float>();
            }
            catch(std::exception &e){
                // res.status = 400;
                // res.set_content(
                    // nlohmann::json({{"error", "Invalid JSON"}, {"what", e.what()}}).dump(),
                    // "application/json"
                // );
                RCLCPP_INFO_STREAM(this->get_logger(), e.what());
                return;
            }
        }
        if(!map_client_->wait_for_action_server(std::chrono::seconds(10))){
            std::lock_guard<std::mutex> lock(map_save_mutex_);
            // res.status = 503;
            map_save_status_ = 0;
            // res.set_content("{\"status\":" + std::to_string(map_save_status_) + "}", "application/json");
            return;
        }

/*         // 异步发送 goal
        auto send_goal_options = rclcpp_action::Client<web_control_msgs::action::MapSave>::SendGoalOptions();
        send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<web_control_msgs::action::MapSave>::WrappedResult &result){
            std::lock_guard<std::mutex> lock(map_save_mutex_);
            if(result.code == rclcpp_action::ResultCode::SUCCEEDED){
                RCLCPP_INFO(this->get_logger(), "444444444444");
                map_save_status_ = 3;
            }
            else{
                map_save_status_ = -1;
                RCLCPP_INFO(this->get_logger(), "555555555555");
            }
        };

        {
            std::lock_guard<std::mutex> lock(map_save_mutex_);
            map_save_status_ = 1; // 正在执行
        }
        map_client_->async_send_goal(goal, send_goal_options);

        RCLCPP_INFO(this->get_logger(), "66666666666");
        // 立即返回
        res.set_content("{\"status\":1}", "application/json"); */

        auto send_goal_options = rclcpp_action::Client<web_control_msgs::action::MapSave>::SendGoalOptions();

        auto goal_handle_future = map_client_->async_send_goal(goal);
        auto goal_handle = goal_handle_future.get();

        if (!goal_handle) {
            map_save_status_ = 0;
            // res.set_content("{\"status\":0}", "application/json");
            return;
        }

        auto result_future = map_client_->async_get_result(goal_handle);
        auto result = result_future.get();
        { 
            if(result.code == rclcpp_action::ResultCode::SUCCEEDED){
                map_save_status_ = 3;
            }
            else if(result.code == rclcpp_action::ResultCode::ABORTED){
                map_save_status_ = -1;
            }
            else if (result.code == rclcpp_action::ResultCode::CANCELED){
                map_save_status_ = -1;
            } 
            else{
                map_save_status_ = -1;
            }

            nlohmann::json resp_json;
            resp_json["status"] = map_save_status_;
            res.set_content(resp_json.dump(), "application/json");
        }           
    });

    svr_->Get("/map_save_status", [this](const httplib::Request&, httplib::Response &res){
        std::lock_guard<std::mutex> lock(map_save_mutex_);
        res.set_content("{\"status\":" + std::to_string(map_save_status_) + "}", "application/json"); 
    });

    svr_->Get("/clear_map", [this](const httplib::Request&, httplib::Response &res){
        std::filesystem::path dir = nav2_bringup_dir_ + "/maps";
        std::regex pattern(R"(map1\..*)");
        nlohmann::json resp_json;
        if(!std::filesystem::exists(dir)){
            resp_json["status"] = -1;
            resp_json["messages"] = "Directory not found";
            res.set_content(resp_json.dump(), "application/json");
            return;
        }

        for(const auto& entry : std::filesystem::directory_iterator(dir)){
            if(std::filesystem::is_regular_file(entry.path())){
                std::string filename = entry.path().filename().string();
                if(std::regex_match(filename, pattern)){
                    try {
                        std::filesystem::remove(entry.path());
                        resp_json["messages"][filename] = "deleted";
                    } catch (const std::filesystem::filesystem_error& e) {
                        resp_json["messages"][filename] = std::string("failed: ") + e.what();
                        resp_json["status"] = -2;
                        res.set_content(resp_json.dump(), "application/json");
                        return;
                    }
                }
            }
        }

        resp_json["status"] = 1;  // 总体状态
        res.set_content(resp_json.dump(), "application/json");

    });

    svr_->Post("/run", [this](const httplib::Request& req, httplib::Response &res){
        nlohmann::json req_json = nlohmann::json::parse(req.body);
        int32_t data_value = req_json.at("data");
        auto request = std::make_shared<web_control_msgs::srv::RunControl::Request>();
        request->data = data_value;
        nlohmann::json res_json;
        res_json["status"] = -1;
        if(!run_client_->wait_for_service(std::chrono::seconds(2))){
            res_json["messages"] = "error: internal service is not ready.";
            res.set_content(res_json.dump(), "application/json"); 
            return;
        }

        auto future = run_client_->async_send_request(request);
        if(future.wait_for(std::chrono::seconds(5)) != std::future_status::ready){
            res_json["messages"] = "internal service is timeout.";
            res.set_content(res_json.dump(), "application/json"); 
            return;
        }
        auto response = future.get();
        res_json["status"] = response->result;
        res_json["messages"] = response->messages;
        res.set_content(res_json.dump(), "application/json"); 
    });
}