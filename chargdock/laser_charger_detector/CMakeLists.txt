cmake_minimum_required(VERSION 3.8)
project(laser_charger_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(pid_controller REQUIRED)
find_package(control_msgs REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



# 添加groove_detector可执行文件
add_executable(groove_detector src/groove_detector.cpp)
set_target_properties(groove_detector PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
ament_target_dependencies(groove_detector
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  pcl_conversions
)
target_link_libraries(groove_detector ${PCL_LIBRARIES})

if(OpenMP_CXX_FOUND)
    target_link_libraries(groove_detector OpenMP::OpenMP_CXX)
endif()


# 添加chassis_controller可执行文件
add_executable(chassis_controller 
  src/chassis_controller.cpp
  src/bezier_curve.cpp
)
set_target_properties(chassis_controller PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
target_include_directories(chassis_controller PRIVATE 
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

# 链接库文件
target_link_libraries(chassis_controller
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)


# 设置依赖关系
ament_target_dependencies(chassis_controller
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  control_toolbox
  pid_controller
  control_msgs
)

install(TARGETS
  groove_detector
  chassis_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

# 安装launch文件夹
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
