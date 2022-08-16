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

#include "cartographer_ros/submap.h"

#include "absl/memory/memory.h"
#include "cartographer/common/port.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/SubmapQuery.h"

namespace cartographer_ros {

/**
 * @brief 获取SubmapTextures
 * 
 * @param[in] submap_id submap的id
 * @param[in] client SubmapQuery服务的客户端
 * @return std::unique_ptr<::cartographer::io::SubmapTextures> 
 */
std::unique_ptr<::cartographer::io::SubmapTextures> FetchSubmapTextures(
    const ::cartographer::mapping::SubmapId& submap_id,
    ros::ServiceClient* client)
{
  // 临时对象，负责组织请求数据
  ::cartographer_ros_msgs::SubmapQuery srv;
  //  初始化客户端请求
  srv.request.trajectory_id = submap_id.trajectory_id;
  srv.request.submap_index = submap_id.submap_index;
  
  // call（）也是一个阻塞型函数，将request发出去之后一直等待直到显示服务器的回应
  // step1 客户端访问服务端，提交请求srv给服务端处理，返回 bool 值，标记是否成功，服务端调用HandleSubmapQuery处理请求，处理完请求后，更新响应结果response
  if (!client->call(srv) ||
      srv.response.status.code != ::cartographer_ros_msgs::StatusCode::OK)// 此时的响应，是服务端已经更新过的
  {
    return nullptr;
  }

  // 如果服务端响应的地图纹理是空
  if (srv.response.textures.empty())
  {
    return nullptr;
  }

  // step2 将srv.response.textures格式的数据 转换成io::SubmapTextures
  // 创建指向压缩后的地图栅格数据的智能指针
  auto response = absl::make_unique<::cartographer::io::SubmapTextures>();
  response->version = srv.response.submap_version;
  // 遍历服务端响应的地图纹理信息
  for (const auto& texture : srv.response.textures)
  {
    // 获取压缩后的地图栅格数据，string格式
    const std::string compressed_cells(texture.cells.begin(),
                                       texture.cells.end());
                                       
    // 将解压后的数据填充到SubmapTexture中，并添加到response的纹理图中
    response->textures.emplace_back(::cartographer::io::SubmapTexture{
        // step3 将地图栅格数据进行解压
        ::cartographer::io::UnpackTextureData(compressed_cells, texture.width,
                                              texture.height),
        texture.width, texture.height, texture.resolution,
        ToRigid3d(texture.slice_pose)});
  }
  // 返回响应
  return response;
}

}  // namespace cartographer_ros
