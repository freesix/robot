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

#include "cartographer/common/configuration_file_resolver.h"

#include <fstream>
#include <iostream>
#include <streambuf>

#include "cartographer/common/config.h"
#include "glog/logging.h"

namespace cartographer {
namespace common {
/**
 * @brief 配置文件solver的构造
 * @param[in] configuration_files_directories 配置文件目录
 */
ConfigurationFileResolver::ConfigurationFileResolver(
    const std::vector<std::string>& configuration_files_directories)
    : configuration_files_directories_(configuration_files_directories) {
  configuration_files_directories_.push_back(kConfigurationFilesDirectory); // 获取cartorgrapher的配置文件目录
}
/**
 * @brief 给定文件名，查找文件夹中是否存在
 * @param[in] 文件名
 * @return 查找到的文件路径
 */
std::string ConfigurationFileResolver::GetFullPathOrDie(
    const std::string& basename) {
  for (const auto& path : configuration_files_directories_) {
    const std::string filename = path + "/" + basename;
    std::ifstream stream(filename.c_str());
    if (stream.good()) {
      LOG(INFO) << "Found '" << filename << "' for '" << basename << "'.";
      return filename;
    }
  }
  LOG(FATAL) << "File '" << basename << "' was not found.";
}
/**
 * @brief 读取配置文件内容
 * @param[in] basname 文件名
 * @return 配置文件中的数据流
 */
std::string ConfigurationFileResolver::GetFileContentOrDie(
    const std::string& basename) {
  CHECK(!basename.empty()) << "File basename cannot be empty." << basename;
  const std::string filename = GetFullPathOrDie(basename); // 根据给定文件名查找是否存在文件夹中
  std::ifstream stream(filename.c_str());
  return std::string((std::istreambuf_iterator<char>(stream)),
                     std::istreambuf_iterator<char>()); // 读取并返回文件内容
}

}  // namespace common
}  // namespace cartographer
