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

#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_H_

#include <memory>

#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/sensor/collator_interface.h"

namespace cartographer {
namespace mapping {

// Wires up the complete SLAM stack with TrajectoryBuilders (for local submaps)
// and a PoseGraph for loop closure.

// 包含前端(TrajectoryBuilders,scan to submap) 与 后端(用于查找回环的PoseGraph) 的完整的SLAM
class MapBuilder : public MapBuilderInterface 
{
 public:
  explicit MapBuilder(const proto::MapBuilderOptions &options);
  ~MapBuilder() override {}

  MapBuilder(const MapBuilder &) = delete;
  MapBuilder &operator=(const MapBuilder &) = delete;

  int AddTrajectoryBuilder(
      const std::set<SensorId> &expected_sensor_ids,
      const proto::TrajectoryBuilderOptions &trajectory_options,
      LocalSlamResultCallback local_slam_result_callback) override;

  int AddTrajectoryForDeserialization(
      const proto::TrajectoryBuilderOptionsWithSensorIds
          &options_with_sensor_ids_proto) override;

  void FinishTrajectory(int trajectory_id) override;

  std::string SubmapToProto(const SubmapId &submap_id,
                            proto::SubmapQuery::Response *response) override;

  void SerializeState(bool include_unfinished_submaps,
                      io::ProtoStreamWriterInterface *writer) override;

  bool SerializeStateToFile(bool include_unfinished_submaps,
                            const std::string &filename) override;

  std::map<int, int> LoadState(io::ProtoStreamReaderInterface *reader,
                               bool load_frozen_state) override;

  std::map<int, int> LoadStateFromFile(const std::string &filename,
                                       const bool load_frozen_state) override;
  // 后端：位姿图，图优化
  mapping::PoseGraphInterface *pose_graph() override 
  {
    return pose_graph_.get();
  }

  int num_trajectory_builders() const override {
    return trajectory_builders_.size();
  }

  // 返回指向CollatedTrajectoryBuilder的指针
  mapping::TrajectoryBuilderInterface *GetTrajectoryBuilder(
      int trajectory_id) const override 
  {
    return trajectory_builders_.at(trajectory_id).get();
  }

  const std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
      &GetAllTrajectoryBuilderOptions() const override {
    return all_trajectory_builder_options_;
  }

 private:
  // 配置文件对象 has-a关系
  const proto::MapBuilderOptions options_;
  // 线程池对象 has-a关系
  common::ThreadPool thread_pool_; 
  // 指向后端的智能指针
  std::unique_ptr<PoseGraph> pose_graph_;
  // 传感器整理器
  std::unique_ptr<sensor::CollatorInterface> sensor_collator_;
  // 容器中存放的是智能指针指向基类TrajectoryBuilderInterface
  std::vector<std::unique_ptr<mapping::TrajectoryBuilderInterface>> trajectory_builders_;

  std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
      all_trajectory_builder_options_;
};

// 工厂函数，智能指针:初始化MapBuilderInterface类型的对象指针
std::unique_ptr<MapBuilderInterface> CreateMapBuilder(const proto::MapBuilderOptions& options);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_H_