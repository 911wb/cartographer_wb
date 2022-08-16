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
 * @brief Construct a new Configuration File Resolver:: Configuration File Resolver object
 * 
 * @param[in] configuration_files_directories 配置文件目录
 */
// lua文件有两个地址一个在src，一个在install_isolated下
// 入参:一个容器，容器中存放的是lua文件的地址(src)
ConfigurationFileResolver::ConfigurationFileResolver(
    const std::vector<std::string>& configuration_files_directories)
    : configuration_files_directories_(configuration_files_directories) 
{
  // 成员变量，添加install_isolated下的lua文件路径
  configuration_files_directories_.push_back(kConfigurationFilesDirectory);
}

/**
 * @brief 在所有的配置文件目录中 根据给定配置文件的名字 搜索 配置文件
 * 
 * @param[in] basename 给定配置文件的名字
 * @return std::string 如果搜索成功, 返回配置文件的全路径名
 */
std::string ConfigurationFileResolver::GetFullPathOrDie(
    const std::string& basename) 
{
  // 遍历两个lua文件夹的路径
  for (const auto& path : configuration_files_directories_) 
  {
    // lua文件的路径+lua文件名
    const std::string filename = path + "/" + basename;
    std::ifstream stream(filename.c_str());
    // 如果能够打开该文件
    if (stream.good()) 
    {
      LOG(INFO) << "---found " << filename << " for " << basename << " ---";
      // lua文件的路径+lua文件名
      return filename;
    }
  }
  // 如果找不到配置文件就退出整个程序
  LOG(FATAL) << "file " << basename << " was not found";
}

/**
 * @brief 读取配置文件内容，并返回
 * 
 * @param[in] basename 文件名
 * @return std::string 文件内容的数据流
 */
std::string ConfigurationFileResolver::GetFileContentOrDie(
    const std::string& basename) 
{
  CHECK(!basename.empty()) << "File basename cannot be empty." << basename;

  // 根据文件名查找是否在 configuration_files_directories_中存在
  // 若存在，则返回：lua文件的路径+lua文件名
  const std::string filename = GetFullPathOrDie(basename);
  // 读取配置文件的内容，放到stream中
  std::ifstream stream(filename.c_str());

  // 把读取到的内容通过string的形式返回
  return std::string((std::istreambuf_iterator<char>(stream)),
                     std::istreambuf_iterator<char>());
}

}  // namespace common
}  // namespace cartographer
