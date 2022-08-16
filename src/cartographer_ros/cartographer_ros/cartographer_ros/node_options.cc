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

#include "cartographer_ros/node_options.h"

#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "glog/logging.h"

namespace cartographer_ros 
{
/**
 * @brief 读取lua文件内容, 将lua文件的内容赋值给NodeOptions
 * 
 * @param lua_parameter_dictionary lua字典
 * @return NodeOptions 
 */
// 传入lua文件内容，以及指向该内容的指针
NodeOptions CreateNodeOptions(
    ::cartographer::common::LuaParameterDictionary* const
        lua_parameter_dictionary)
{
  // 创建一个对象存放部分lua文件参数，以下是给对象的属性赋值
  NodeOptions options;

  // 根据lua字典中的参数, 生成protobuf的序列化数据结构 proto::MapBuilderOptions
  // 获取构建轨迹的参数
  options.map_builder_options =
      ::cartographer::mapping::CreateMapBuilderOptions(
          lua_parameter_dictionary->GetDictionary("map_builder").get());

  // 输入键，在lua文件中找到对应的值，把值赋给对象的属性
  options.map_frame = lua_parameter_dictionary->GetString("map_frame");
  options.lookup_transform_timeout_sec =
      lua_parameter_dictionary->GetDouble("lookup_transform_timeout_sec");
  options.submap_publish_period_sec =
      lua_parameter_dictionary->GetDouble("submap_publish_period_sec");
  options.pose_publish_period_sec =
      lua_parameter_dictionary->GetDouble("pose_publish_period_sec");
  options.trajectory_publish_period_sec =
      lua_parameter_dictionary->GetDouble("trajectory_publish_period_sec");
  if (lua_parameter_dictionary->HasKey("publish_to_tf")) {
    options.publish_to_tf =
        lua_parameter_dictionary->GetBool("publish_to_tf");
  }
  if (lua_parameter_dictionary->HasKey("publish_tracked_pose")) {
    options.publish_tracked_pose =
        lua_parameter_dictionary->GetBool("publish_tracked_pose");
  }
  if (lua_parameter_dictionary->HasKey("use_pose_extrapolator")) {
    options.use_pose_extrapolator =
        lua_parameter_dictionary->GetBool("use_pose_extrapolator");
  }

  // 返回赋值后的对象options
  return options;
}

/**
 * @brief 加载lua配置文件中的参数
 * 
 * @param[in] configuration_directory 配置文件所在地址
 * @param[in] configuration_basename 配置文件的名字
 * @return std::tuple<NodeOptions, TrajectoryOptions> 返回节点的配置与轨迹的配置
 */

std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename)
{
  // 返回智能指针，获取配置文件所在的目录，花括号用来初始化容器
  auto file_resolver =
        // 调用构造函数初始化智能指针(指针对象)，指向ConfigurationFileResolver
      absl::make_unique<cartographer::common::ConfigurationFileResolver>(
        std::vector<std::string>{configuration_directory});
        
  // 返回配置文件(lua文件)的所有内容，以字符串形式赋值给code
  const std::string code =
      file_resolver->GetFileContentOrDie(configuration_basename);

  // 构造函数初始化：lua文件的内容(字符串类型)，及其地址(智能指针)，复杂的lua文件操作生成lua字典
  cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

  // 创建元组tuple,元组定义了一个有固定数目元素的容器, 其中的每个元素类型都可以不相同
  // 返回赋值后的对象options，组合成元组，make_tuple(NodeOptions options, TrajectoryOptions options)
  return std::make_tuple(CreateNodeOptions(&lua_parameter_dictionary),
                        CreateTrajectoryOptions(&lua_parameter_dictionary));
}

}  // namespace cartographer_ros
