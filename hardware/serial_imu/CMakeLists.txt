cmake_minimum_required(VERSION 3.8)
project(serial_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE "Debug")
# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

include_directories("include/serial_imu")

aux_source_directory(src/ SOURCE)
ament_auto_add_library(${PROJECT_NAME}_lib ${SOURCE})

add_executable(${PROJECT_NAME}_node main_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

add_executable(${PROJECT_NAME}_view view_node.cpp)
target_link_libraries(${PROJECT_NAME}_view ${PROJECT_NAME}_lib)

install(TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_view
  DESTINATION lib/${PROJECT_NAME} 
)

install(TARGETS
  ${PROJECT_NAME}_lib
  DESTINATION lib/${PROJECT_NAME} 
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_auto_package()
