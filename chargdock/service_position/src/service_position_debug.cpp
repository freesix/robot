#include "rclcpp/rclcpp.hpp"
#include "service_position_msgs/srv/set_position.hpp"
#include "service_position_msgs/srv/get_position.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("service_position_debug_client");

    // 创建set_position客户端
    auto set_client = node->create_client<service_position_msgs::srv::SetPosition>("set_position");
    // 创建get_position客户端
    auto get_client = node->create_client<service_position_msgs::srv::GetPosition>("get_position");

    // 等待服务可用
    if (!set_client->wait_for_service(3s)) {
        RCLCPP_ERROR(node->get_logger(), "set_position服务不可用");
        return 1;
    }
    if (!get_client->wait_for_service(3s)) {
        RCLCPP_ERROR(node->get_logger(), "get_position服务不可用");
        return 1;
    }

    // 调用set_position
    auto set_req = std::make_shared<service_position_msgs::srv::SetPosition::Request>();
    set_req->x = 0.0; // 设置x坐标
    set_req->y = 0.0; // 设置y坐标
    auto set_result = set_client->async_send_request(set_req);
    // 等待响应
    if (rclcpp::spin_until_future_complete(node, set_result) == rclcpp::FutureReturnCode::SUCCESS) {
        auto resp = set_result.get();
        RCLCPP_INFO(node->get_logger(), "set_position响应: success=%d, message=%s", resp->success, resp->message.c_str());
    } else {
        RCLCPP_ERROR(node->get_logger(), "set_position调用失败");
    }

    // 调用get_position
    auto get_req = std::make_shared<service_position_msgs::srv::GetPosition::Request>();
    // GetPosition请求为空，不需要设置参数
    auto get_result = get_client->async_send_request(get_req);
    if (rclcpp::spin_until_future_complete(node, get_result) == rclcpp::FutureReturnCode::SUCCESS) {
        auto resp = get_result.get();
        RCLCPP_INFO(node->get_logger(), "get_position响应: success=%d, message=%s", resp->success, resp->message.c_str());
    } else {
        RCLCPP_ERROR(node->get_logger(), "get_position调用失败");
    }

    rclcpp::shutdown();
    return 0;
} 