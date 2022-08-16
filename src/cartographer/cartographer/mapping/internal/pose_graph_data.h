/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_POSE_GRAPH_DATA_H_
#define CARTOGRAPHER_MAPPING_POSE_GRAPH_DATA_H_

#include <map>
#include <set>
#include <vector>

#include "cartographer/mapping/internal/optimization/optimization_problem_2d.h"
#include "cartographer/mapping/internal/optimization/optimization_problem_3d.h"
#include "cartographer/mapping/internal/trajectory_connectivity_state.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/submaps.h"

namespace cartographer {
namespace mapping {

// The current state of the submap in the background threads. After this
// transitions to 'kFinished', all nodes are tried to match
// against this submap. Likewise, all new nodes are matched against submaps in
// that state.
// 后台线程中子图的当前状态.在此转换为“kFinished”之后, 所有节点都将尝试与此子图匹配, 进行回环检测. 
// 同样, 所有新节点都与kNoConstraintSearch状态下的子图匹配.
enum class SubmapState { kNoConstraintSearch, kFinished };

// 轨迹的状态
struct InternalTrajectoryState {
  enum class DeletionState {
    NORMAL,
    SCHEDULED_FOR_DELETION,
    WAIT_FOR_DELETION
  };

  PoseGraphInterface::TrajectoryState state =
      PoseGraphInterface::TrajectoryState::ACTIVE;
  DeletionState deletion_state = DeletionState::NORMAL;
};

// 保存的子图的指针与属于这张子图的节点的id
struct InternalSubmapData 
{
  // 指向子图的指针
  std::shared_ptr<const Submap> submap;
  SubmapState state = SubmapState::kNoConstraintSearch;

  // IDs of the nodes that were inserted into this map together with
  // constraints for them. They are not to be matched again when this submap
  // becomes 'kFinished'.
  // 插入到此地图中的节点的 ID
  // 当此子图变为“kFinished”后, 这些节点将不再与这个地图进行匹配.
  std::set<NodeId> node_ids;
};

// 该数据结构储存的是所有需要优化和保存的数据
struct PoseGraphData 
{
  // Submaps get assigned an ID and state as soon as they are seen, even
  // before they take part in the background computations.

  // submap_data_ 里面,包含了所有的submap <键, 值>
  MapById<SubmapId, InternalSubmapData> submap_data;

  // submap 在 global 坐标系下的坐标 <子图id, 子图在global坐标系下的所有位姿>
  MapById<SubmapId, optimization::SubmapSpec2D> global_submap_poses_2d;

  MapById<SubmapId, optimization::SubmapSpec3D> global_submap_poses_3d;

  // notice: 所有的轨迹节点的id 与 节点的在global坐标系下的坐标, 在local map 下的坐标与时间
  MapById<NodeId, TrajectoryNode> trajectory_nodes;

  // Global landmark poses with all observations.
  std::map<std::string /* landmark ID */, PoseGraphInterface::LandmarkNode>
      landmark_nodes;

  // How our various trajectories are related.
  TrajectoryConnectivityState trajectory_connectivity_state;
  // 节点的个数
  int num_trajectory_nodes = 0;
  // 轨迹id与轨迹的状态(活跃、完成、冻结、删除)
  std::map<int, InternalTrajectoryState> trajectories_state;

  // Set of all initial trajectory poses.
  // <轨迹id, 初始位姿>
  std::map<int, PoseGraph::InitialTrajectoryPose> initial_trajectory_poses;

  // 所有的约束数据(子图内约束、子图间约束)
  std::vector<PoseGraphInterface::Constraint> constraints;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_GRAPH_DATA_H_
