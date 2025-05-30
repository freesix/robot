/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros {
namespace {

void Run() {
  rclcpp::Node::SharedPtr cartographer_node = rclcpp::Node::make_shared("cartographer_node");
  constexpr double kTfBufferCacheTimeInSeconds = 10.; // 常量，且在编译时计算出值
  // 创建一个tf2_ros::Buffer的指针对象，缓存tf数据，缓存时间段为10秒
  std::shared_ptr<tf2_ros::Buffer> tf_buffer =
      std::make_shared<tf2_ros::Buffer>(
        cartographer_node->get_clock(),
        tf2::durationFromSec(kTfBufferCacheTimeInSeconds),
        cartographer_node);
  // 创建一个tf监听器，将变换信息存储到tf_buffer中
  std::shared_ptr<tf2_ros::TransformListener> tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  // 解析配置，分别放到node_options和trajectory_options中
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) =  // std::tie只接受元组返回
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
  // 初始化建图接口
  auto map_builder =
    cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  // 申明一个cartographer_ros::Node对象，将ROS2的topic数据传入cartogarpher(MapBuilder)
  auto node = std::make_shared<cartographer_ros::Node>(
    node_options, std::move(map_builder), tf_buffer, cartographer_node,
    FLAGS_collect_metrics);
  // 加载序列化文件.pdstream中的slam状态，具体是那些需要看代码
  if (!FLAGS_load_state_filename.empty()) {
    node->LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }
  // 使用默认topic开始轨迹生成
  if (FLAGS_start_trajectory_with_default_topics) {
    node->StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  rclcpp::spin(cartographer_node);
  // 结束所有处于活动状态的轨迹
  node->FinishAllTrajectories();
  // 当所有的轨迹结束时，再执行一次全局优化
  node->RunFinalOptimization();
  // 保存slam序列化信息
  if (!FLAGS_save_state_filename.empty()) {
    node->SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  // Init rclcpp first because gflags reorders command line flags in argv
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing(); // 允许程序在运行时重新解析命令行参数
  google::InitGoogleLogging(argv[0]); // 初始化glog
  google::ParseCommandLineFlags(&argc, &argv, false); // 解析gflags的标志命令

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing."; // 检查配置文件夹是否存在
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing."; // 检查配置文件

  cartographer_ros::ScopedRosLogSink ros_log_sink; // 将glog的日志重定向到ros日志系统中
  cartographer_ros::Run();
  ::rclcpp::shutdown();
}
