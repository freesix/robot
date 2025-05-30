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

#include "cartographer_ros/ros_log_sink.h"

#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#include "glog/log_severity.h"
#include "glog/logging.h"

namespace cartographer_ros {

namespace {
/**
 * @brief 根据给定文件全路径名，获取文件名字
 * @param[in] filepath 文件全路径字符串
 * @return 返回文件名字
 */
const char* GetBasename(const char* filepath) {
  const char* base = std::strrchr(filepath, '/');
  return base ? (base + 1) : filepath;
}

}  // namespace
/**
 * @brief 调用AddLogSink()函数，将ScopedRosLogSink类注册到glog中
 */
ScopedRosLogSink::ScopedRosLogSink() : will_die_(false) { AddLogSink(this); }
/**
 * @brief 析构函数，调用了RemoveLogSink()函数
 */
ScopedRosLogSink::~ScopedRosLogSink() { RemoveLogSink(this); }
/**
 * @brief 重载了glog中的send()方法，使用ROS_INFO进行glog消息的输出
 * @param[in] severity 消息级别
 * @param[in] filename 全路径文件名
 * @param[in] base_filename 文件名
 * @param[in] line 消息所在文件行数
 * @param[in] tm_time 消息的时间
 * @param[in] message 消息的本体
 * @param[in] message_len 消息长度
 */
void ScopedRosLogSink::send(const ::google::LogSeverity severity,
                            const char* const filename,
                            const char* const base_filename, const int line,
                            const struct std::tm* const tm_time,
                            const char* const message,
                            const size_t message_len) {
  (void) base_filename; // TODO: remove unused arg ?

  // Google glog broke the ToString API, but has no way to tell what version it is using.
  // To support both newer and older glog versions, use a nasty hack were we infer the
  // version based on whether GOOGLE_GLOG_DLL_DECL is defined
#if defined(GOOGLE_GLOG_DLL_DECL)
  const std::string message_string = ::google::LogSink::ToString(
      severity, GetBasename(filename), line, tm_time, message, message_len);
#else
  const std::string message_string = ::google::LogSink::ToString(
      severity, GetBasename(filename), line, ::google::LogMessageTime(*tm_time), message, message_len);
#endif
  switch (severity) {
    case ::google::GLOG_INFO:
      RCLCPP_INFO_STREAM(logger_, message_string);
      break;

    case ::google::GLOG_WARNING:
      RCLCPP_WARN_STREAM(logger_, message_string);
      break;

    case ::google::GLOG_ERROR:
      RCLCPP_ERROR_STREAM(logger_, message_string);
      break;

    case ::google::GLOG_FATAL:
      RCLCPP_FATAL_STREAM(logger_, message_string);
      will_die_ = true;
      break;
  }
}
/**
 * @brief 当glog输出FATAL级别消息，will_die_为true，给ros一些时间暂定，用于异步的情况
 */
void ScopedRosLogSink::WaitTillSent() {
  if (will_die_) {
    // Give ROS some time to actually publish our message.
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

}  // namespace cartographer_ros
