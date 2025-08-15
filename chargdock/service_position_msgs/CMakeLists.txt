cmake_minimum_required(VERSION 3.8)
project(service_position_msgs)

# 查找必要的包
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 声明要生成的服务消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetPosition.srv"
  "srv/SetPosition.srv"
  DEPENDENCIES std_msgs
)

# 导出依赖
ament_export_dependencies(rosidl_default_runtime) 

install(
  DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 