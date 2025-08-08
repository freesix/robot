#include "httpface/httpserver.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HttpServer>(1234);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}