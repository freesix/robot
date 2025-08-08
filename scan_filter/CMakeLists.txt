cmake_minimum_required(VERSION 3.8)
project(scan_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(laser_scan_filter src/main.cpp)
ament_target_dependencies(laser_scan_filter rclcpp sensor_msgs)

install(TARGETS laser_scan_filter
  DESTINATION lib/${PROJECT_NAME})
ament_package()
