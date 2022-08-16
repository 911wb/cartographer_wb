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

#include "cartographer/mapping/internal/2d/pose_graph_2d.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include "Eigen/Eigenvalues"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/internal/2d/overlapping_submaps_trimmer_2d.h"
#include "cartographer/mapping/proto/pose_graph/constraint_builder_options.pb.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

static auto* kWorkQueueDelayMetric = metrics::Gauge::Null();
static auto* kWorkQueueSizeMetric = metrics::Gauge::Null();
static auto* kConstraintsSameTrajectoryMetric = metrics::Gauge::Null();
static auto* kConstraintsDifferentTrajectoryMetric = metrics::Gauge::Null();
static auto* kActiveSubmapsMetric = metrics::Gauge::Null();
static auto* kFrozenSubmapsMetric = metrics::Gauge::Null();
static auto* kDeletedSubmapsMetric = metrics::Gauge::Null();

/**
 * @brief 构造函数
 * 
 * @param[in] options 位姿图的参数配置
 * @param[in] optimization_problem 优化问题
 * @param[in] thread_pool map_builder中构造的线程池
 */
PoseGraph2D::PoseGraph2D(
    const proto::PoseGraphOptions& options,
    std::unique_ptr<optimization::OptimizationProblem2D> optimization_problem,
    common::ThreadPool* thread_pool)
    : options_(options),
      optimization_problem_(std::move(optimization_problem)),
      constraint_builder_(options_.constraint_builder_options(), thread_pool),
      thread_pool_(thread_pool)
  {
  // overlapping_submaps_trimmer_2d 在配置文件中被注释掉了, 没有使用
  if (options.has_overlapping_submaps_trimmer_2d()) 
  {
    const auto& trimmer_options = options.overlapping_submaps_trimmer_2d();
    AddTrimmer(absl::make_unique<OverlappingSubmapsTrimmer2D>(
        trimmer_options.fresh_submaps_count(),
        trimmer_options.min_covered_area(),
        trimmer_options.min_added_submaps_count()));
  }
}

// 等待所有计算结束之后再析构
PoseGraph2D::~PoseGraph2D() {
  WaitForAllComputations();
  absl::MutexLock locker(&work_queue_mutex_);
  CHECK(work_queue_ == nullptr);
}

// 返回指定轨迹id下的正处于活跃状态下的子图的SubmapId
// 计算子图在global坐标系下的位姿： T_sumap2global，返回子图id
std::vector<SubmapId> PoseGraph2D::InitializeGlobalSubmapPoses(
    const int trajectory_id, const common::Time time,
    const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps) 
{

  CHECK(!insertion_submaps.empty());

  // submap_data中存的: key：SubmapId后端id, values：对应id的Submap在global坐标系下的全局位姿
  // 包含优化后和没有优化的子图在global坐标系下的pose
  const auto& submap_data = optimization_problem_->submap_data();
  
  // 只有slam刚启动时
  // 第一次执行后端，子图的个数才为1
  if (insertion_submaps.size() == 1)
  {
    // If we don't already have an entry for the first submap, add one.
    // 如果判断指定id的submap_data的size为0, 这条轨迹上还没有添加submap的pose
    if (submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0) 
    {
      // 如果没设置初始位姿就是0, 设置了就是1
      if (data_.initial_trajectory_poses.count(trajectory_id) > 0) 
      {
        // 把该trajectory_id与其初始位姿的基准轨迹的id关联起来
        data_.trajectory_connectivity_state.Connect(
            trajectory_id,
            data_.initial_trajectory_poses.at(trajectory_id).to_trajectory_id,
            time);
      }
      // 将该submap的global pose加入到optimization_problem_中
      // T_submap2global * T_submap2local¯¹ * T_submap2local = T_loccal2global * T_submap2local = T_submap2global
      optimization_problem_->AddSubmap(
          trajectory_id, transform::Project2D(
                             ComputeLocalToGlobalTransform(
                                 data_.global_submap_poses_2d, trajectory_id) *
                             insertion_submaps[0]->local_pose()));
    }

    CHECK_EQ(1, submap_data.SizeOfTrajectoryOrZero(trajectory_id));

    // 因为是第一个submap, 所以该submap的ID是(trajectory_id,0), 其中0是submap的index, 从0开始
    const SubmapId submap_id{trajectory_id, 0};
    // 检查这个SubmapId下的submap是否等于insertion_submaps的第一个元素.因为我们初始化第一个submap肯定是要被插入的那个submap
    CHECK(data_.submap_data.at(submap_id).submap == insertion_submaps.front());
    // 因为是第一个submap, 那就把刚刚建立的submap的id返回
    return {submap_id};// 花括号对容器初始化
  }

  CHECK_EQ(2, insertion_submaps.size());

  // 获取 submap_data 的末尾 trajectory_id
  const auto end_it = submap_data.EndOfTrajectory(trajectory_id);
  CHECK(submap_data.BeginOfTrajectory(trajectory_id) != end_it);

  // end_it是最后一个元素的下一个位置, 所以它之前的一个submap的id就是submap_data中的最后一个元素
  // 注意, 这里的last_submap_id 是 optimization_problem_->submap_data() 中的
  const SubmapId last_submap_id = std::prev(end_it)->id;

  // 如果是等于第一个子图, 说明insertion_submaps的第二个子图还没有加入到optimization_problem_中，那就加入
  // 拿着optimization_problem_中子图的索引, 根据这个索引在data_.submap_data中获取地图的指针
  if (data_.submap_data.at(last_submap_id).submap == insertion_submaps.front()) 
  {
    // In this case, 'last_submap_id' is the ID of
    // 'insertions_submaps.front()' and 'insertions_submaps.back()' is new.
    
    // 这种情况下, 要给新的submap分配id, 并把它加到OptimizationProblem的submap_data_这个容器中
    // T_submap₁2global，只有一个子图的时候，最后一个就是第一个
    const auto& first_submap_pose = submap_data.at(last_submap_id).global_pose;

    // 解算新的submap的global pose, 插入到OptimizationProblem2D::submap_data_中
    optimization_problem_->AddSubmap(
        trajectory_id,
        // T_submap₁2global * T_submap₁2local¯¹ * T_submap₂2local = T_local2global *  T_submap₂2local = T_submap₂2global
        first_submap_pose *
            constraints::ComputeSubmapPose(*insertion_submaps[0]).inverse() *
            constraints::ComputeSubmapPose(*insertion_submaps[1]));

    return {last_submap_id,
            SubmapId{trajectory_id, last_submap_id.submap_index + 1}};
  }

  // 如果是等于第二个子图, 说明第二个子图已经分配了id, 已经在OptimizationProblem的submap_data_中了
  CHECK(data_.submap_data.at(last_submap_id).submap ==
        insertion_submaps.back());
  // 那么第一个子图的index就是last_submap_id.submap_index的前一个, 所以要-1
  const SubmapId front_submap_id{trajectory_id,
                                 last_submap_id.submap_index - 1};
  CHECK(data_.submap_data.at(front_submap_id).submap ==
        insertion_submaps.front());
  // 返回第一个与第二个子图的id
  return {front_submap_id, last_submap_id};
}

/**
 *
 * @brief 向节点后验位姿列表中添加一个新的节点后验位姿, 并保存新生成的submap
 * 
 * @param[in] constant_data 后验位姿、一帧体素滤波后重力对齐后的点云、gravity_alignment
 * @param[in] trajectory_id 轨迹id
 * @param[in] insertion_submaps 点云写入的子图
 * @param[in] optimized_pose  Pose_track2global
 * @return NodeId 返回新生成的节点id
 */
// 与顶层接口基本一致，增加了该节点在整个全局地图的全局绝对位置，也是后期需要优化的位姿。
NodeId PoseGraph2D::AppendNode(
    std::shared_ptr<const TrajectoryNode::Data> constant_data,
    const int trajectory_id,
    const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps,
    const transform::Rigid3d& optimized_pose) 
{
  absl::MutexLock locker(&mutex_);

  // 如果轨迹不存在, 则将轨迹添加到连接状态里并添加采样器
  AddTrajectoryIfNeeded(trajectory_id);

  // 根据轨迹状态判断是否可以添加任务
  if (!CanAddWorkItemModifying(trajectory_id)) 
  {
    LOG(WARNING) << "AddNode was called for finished or deleted trajectory.";
  }

  // notice: 向位姿节点列表中添加一个新的节点位姿
  const NodeId node_id = data_.trajectory_nodes.Append(
      trajectory_id, TrajectoryNode{constant_data, optimized_pose});
  
  // 节点总个数加1
  ++data_.num_trajectory_nodes;

  // Test if the 'insertion_submap.back()' is one we never saw before.
  // 如果是刚开始的轨迹, 或者insertion_submaps.back()是第一次看到, 就添加新的子图
  if (data_.submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0 ||
      std::prev(data_.submap_data.EndOfTrajectory(trajectory_id))
              ->data.submap != insertion_submaps.back())// submap2 != submap3
  {
    // We grow 'data_.submap_data' as needed. This code assumes that the first
    // time we see a new submap is as 'insertion_submaps.back()'.

    // 如果insertion_submaps.back()是第一次看到, 也就是新生成的
    // 在data_.submap_data中加入一个空的InternalSubmapData数据结构
    const SubmapId submap_id =
        data_.submap_data.Append(trajectory_id, InternalSubmapData());
    
    // notice: 保存前端子图：子图是刚生成的, 但是子图会在前端部分通过插入点云数据进行更新, 这里只保存共享指针
    // 后端不更新子图(只保存子图)，前端更新操作，前端生成完一张完整子图后，就删除该指针，该子图无法再改变(如果建错也无法修改)
    data_.submap_data.at(submap_id).submap = insertion_submaps.back();
    LOG(INFO) << "Inserted submap " << submap_id << ".";
    kActiveSubmapsMetric->Increment();
  }
  return node_id;
}

/**
 * @brief 增加节点, 并计算跟这个节点相关的约束
 * 
 * @param[in] constant_data 当前机器人信息：后验位姿、一帧体素滤波后重力对齐后的点云、gravity_alignment
 * @param[in] trajectory_id 轨迹id
 * @param[in] insertion_submaps 子地图 点云数据写入后的active_submaps
 * @return NodeId 返回节点的ID
 */
NodeId PoseGraph2D::AddNode(
    std::shared_ptr<const TrajectoryNode::Data> constant_data,
    const int trajectory_id,
    const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps) 
{
  // 将节点在local坐标系下的后验位姿 转成 global坐标系下的后验位姿 T_local2global * Pose_track2local = Pose_track2global
  const transform::Rigid3d optimized_pose(
      GetLocalToGlobalTransform(trajectory_id) * constant_data->local_pose);

  // 向节点后验位姿列表加入当前节点的后验位姿
  const NodeId node_id = AppendNode(constant_data,// 后验位姿、一帧体素滤波后重力对齐后的点云、gravity_alignment
                                    trajectory_id,// 轨迹id
                                    insertion_submaps,// 点云写入的子图
                                    optimized_pose);// 一帧点云恢复的后验位姿： Pose_track2global


  // 查看维护的submaps的front是否插入完成（两个submap，front用于匹配和更新的） 如果一个submap finished，则需要进行闭环检测
  // 获取容器中的第一个submap是否是完成状态
  const bool newly_finished_submap =
      insertion_submaps.front()->insertion_finished();
  /*
  最后通过lambda表达式和函数AddWorkItem注册一个为新增节点添加约束的任务ComputeConstraintsForNode。
  根据Cartographer的思想，在该任务下应当会将新增的节点与所有已经处于kFinished状态的子图进行一次匹配建立可能存在的闭环约束。
  此外，当有新的子图进入kFinished状态时，还会将之与所有的节点进行一次匹配。所以这里会通过insertion_submaps.front()来查询旧图的更新状态。

  */
  // lambda表达式作为参数传入到AddWorkItem中，把计算约束的任务函数放入任务队列中等待执行
  AddWorkItem([=]() LOCKS_EXCLUDED(mutex_)
  { 
    // lambda表达式 work_item
    return ComputeConstraintsForNode(node_id,// 当前节点位姿的id
                                    insertion_submaps,// 前端子图
                                    newly_finished_submap);// 容器中的第一个submap是否是完成状态
  });
  return node_id;
}

// 将任务放入到任务队列中等待被执行，入参：任务函数，即lambda表达式
void PoseGraph2D::AddWorkItem(
    const std::function<WorkItem::Result()>& work_item) 
{
  if(false)
  {

  absl::MutexLock locker(&work_queue_mutex_);
  // 如果任务队列没有初始化
  if (work_queue_ == nullptr) 
  {
    // work_queue_的初始化
    work_queue_ = absl::make_unique<WorkQueue>();
    // 创建一个空的任务,Task类型的智能指针
    auto task = absl::make_unique<common::Task>();

    // 参数为lambda表达式，DrainWorkQueue不断执行work_queue_中的函数
    task->SetWorkItem([this]()
    {
      DrainWorkQueue();
    });

    // 将任务放到线程池中等待调度
    thread_pool_->Schedule(std::move(task));
  }

  const auto now = std::chrono::steady_clock::now();

  // notice: 将传入的任务函数放入任务队列中等待执行
  work_queue_->push_back({now, work_item});

  kWorkQueueSizeMetric->Set(work_queue_->size());
  kWorkQueueDelayMetric->Set(
      std::chrono::duration_cast<std::chrono::duration<double>>(
          now - work_queue_->front().time)
          .count());
  }
  
}

// 如果轨迹不存在, 则将轨迹添加到连接状态里并添加采样器
void PoseGraph2D::AddTrajectoryIfNeeded(const int trajectory_id) {
  // 如果不存在就添加map中
  data_.trajectories_state[trajectory_id];

  CHECK(data_.trajectories_state.at(trajectory_id).state !=
        TrajectoryState::FINISHED);
  CHECK(data_.trajectories_state.at(trajectory_id).state !=
        TrajectoryState::DELETED);
  CHECK(data_.trajectories_state.at(trajectory_id).deletion_state ==
        InternalTrajectoryState::DeletionState::NORMAL);

  // 将轨迹添加到连接状态里,并与自己做连接
  data_.trajectory_connectivity_state.Add(trajectory_id);

  // Make sure we have a sampler for this trajectory.
  // 为轨迹添加采样器
  if (!global_localization_samplers_[trajectory_id]) {
    global_localization_samplers_[trajectory_id] =
        absl::make_unique<common::FixedRatioSampler>(
            options_.global_sampling_ratio());
  }
}

void PoseGraph2D::AddImuData(const int trajectory_id,
                             const sensor::ImuData& imu_data) 
{
  AddWorkItem([=]() LOCKS_EXCLUDED(mutex_)
  {
    absl::MutexLock locker(&mutex_);
    if (CanAddWorkItemModifying(trajectory_id)) 
    {
      // 把imu数据加入到优化问题中，并且把任务函数放入到任务队列中等待执行
      optimization_problem_->AddImuData(trajectory_id, imu_data);
    }
    // 添加数据后不用立刻执行全局优化
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

// 把里程计数据加入到优化问题中 这个任务放入到任务队列中
void PoseGraph2D::AddOdometryData(const int trajectory_id,
                                  const sensor::OdometryData& odometry_data) 
{
  AddWorkItem([=]() LOCKS_EXCLUDED(mutex_) 
  {
    absl::MutexLock locker(&mutex_);
    if (CanAddWorkItemModifying(trajectory_id)) 
    {
      optimization_problem_->AddOdometryData(trajectory_id, odometry_data);
    }
    // 添加数据后不用立刻执行全局优化
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

// 把gps数据加入到优化问题中 这个任务放入到任务队列中
void PoseGraph2D::AddFixedFramePoseData(
    const int trajectory_id,
    const sensor::FixedFramePoseData& fixed_frame_pose_data) 
{
  AddWorkItem([=]() LOCKS_EXCLUDED(mutex_) 
  {
    absl::MutexLock locker(&mutex_);
    if (CanAddWorkItemModifying(trajectory_id)) 
    {
      optimization_problem_->AddFixedFramePoseData(trajectory_id, fixed_frame_pose_data);
    }
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

// 把landmark数据加入到data_.landmark_nodes中 这个任务放入到任务队列中
void PoseGraph2D::AddLandmarkData(int trajectory_id,
                                  const sensor::LandmarkData& landmark_data) {
  AddWorkItem([=]() LOCKS_EXCLUDED(mutex_) 
  {
    absl::MutexLock locker(&mutex_);
    if (CanAddWorkItemModifying(trajectory_id)) 
    {
      for (const auto& observation : landmark_data.landmark_observations) 
      {
        data_.landmark_nodes[observation.id].landmark_observations.emplace_back(
            PoseGraphInterface::LandmarkNode::LandmarkObservation{
                trajectory_id, landmark_data.time,
                observation.landmark_to_tracking_transform,
                observation.translation_weight, observation.rotation_weight});
      }
    }
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

/**
 * @brief 进行子图间约束计算, 也可以说成是回环检测
 * 
 * @param[in] node_id 节点的id
 * @param[in] submap_id submap的id
 */
void PoseGraph2D::ComputeConstraint(const NodeId& node_id,
                                    const SubmapId& submap_id) 
{
  bool maybe_add_local_constraint = false;
  bool maybe_add_global_constraint = false;
  const TrajectoryNode::Data* constant_data;
  const Submap2D* submap;

  {
    // 只锁住括号中的内容
    absl::MutexLock locker(&mutex_);
    CHECK(data_.submap_data.at(submap_id).state == SubmapState::kFinished);
    // 如果是未完成状态的地图不进行约束计算
    if (!data_.submap_data.at(submap_id).submap->insertion_finished()) 
    {
      // Uplink server only receives grids when they are finished, so skip
      // constraint search before that.
      return;
    }

    // 获取该 node 和该 submap 中的 node 中较新的时间
    const common::Time node_time = GetLatestNodeTime(node_id, submap_id);
    
    // 两个轨迹的最后连接时间
    const common::Time last_connection_time =
        data_.trajectory_connectivity_state.LastConnectionTime(
            node_id.trajectory_id, submap_id.trajectory_id);

    // 如果节点和子图属于同一轨迹, 或者时间小于阈值
    // 则只需进行 局部搜索窗口 的约束计算(对局部子图进行回环检测)
    if (node_id.trajectory_id == submap_id.trajectory_id || // 只有纯定位时两个id才不等，建图时相等
        node_time < last_connection_time +
                common::FromSeconds(
                    options_.global_constraint_search_after_n_seconds()))// 纯定位时多久执行一次全子图约束：10s
    {
      // 如果节点和子图属于同一条轨迹，或者如果最近有一个全局约束将该节点的轨迹与子图的轨迹联系起来，则只需对局部搜索窗口进行约束匹配即可
      // 局部约束，节点与子图进行匹配
      maybe_add_local_constraint = true;
    }
    // 如果节点与子图不属于同一条轨迹(纯定位) 并且 间隔了一段时间, 同时采样器为true， 才进行 全局搜索窗口 的约束计算(对整体子图进行回环检测)
    else if (global_localization_samplers_[node_id.trajectory_id]->Pulse())// 采样器的目的是减少计算量
    {
      // 全局约束
      maybe_add_global_constraint = true;
    }

    // 获取节点信息数据与地图数据
    constant_data = data_.trajectory_nodes.at(node_id).constant_data.get();
    submap = static_cast<const Submap2D*>(
        data_.submap_data.at(submap_id).submap.get());
  } // end {}

  // 建图时只会执行这块, 通过局部搜索进行回环检测
  if (maybe_add_local_constraint)
  {
    // T_submap2global¯¹ * T_track2global = T_global2submap * T_track2global = T_track2submap
    const transform::Rigid2d initial_relative_pose =
        optimization_problem_->submap_data()
            .at(submap_id).global_pose.inverse() *
        optimization_problem_->node_data().at(node_id).global_pose_2d;

    // 进行局部搜索窗口的约束计算 (对局部子图进行回环检测)
    constraint_builder_.MaybeAddConstraint(
        submap_id, submap, node_id, constant_data, initial_relative_pose);
  }
  // 纯定位时才有可能执行这块，计算全局子图约束
  else if (maybe_add_global_constraint)
  {
    // 全局搜索窗口 的约束计算 (对整体子图进行回环检测)
    constraint_builder_.MaybeAddGlobalConstraint(submap_id, submap, node_id,
                                                 constant_data);
  }
}

/**
 * @brief 保存节点, 计算子图内约束, 查找回环
 * 
 * @param[in] node_id 刚加入的节点位姿ID
 * @param[in] insertion_submaps active_submaps
 * @param[in] newly_finished_submap 容器中的第一个submap是否是完成状态
 * @return WorkItem::Result 是否需要执行全局优化
 */
WorkItem::Result PoseGraph2D::ComputeConstraintsForNode(
    const NodeId& node_id,
    std::vector<std::shared_ptr<const Submap2D>> insertion_submaps,
    const bool newly_finished_submap)
{
  std::vector<SubmapId> submap_ids;                 // 活跃状态下的子图的id
  std::vector<SubmapId> finished_submap_ids;        // 处于完成状态的子图id的集合
  std::set<NodeId> newly_finished_submap_node_ids;  // 刚刚完成的子图对应的节点id
  // 保存节点与计算子图内约束
  {
    // 该锁只作用域括号内部
    absl::MutexLock locker(&mutex_);

    // 获取当前节点信息数据
    const auto& constant_data =
        data_.trajectory_nodes.at(node_id).constant_data;
    
    // 获取 trajectory_id 下的正处于活跃状态下的子图的SubmapId
    submap_ids = InitializeGlobalSubmapPoses(
        node_id.trajectory_id, constant_data->time, insertion_submaps);
    CHECK_EQ(submap_ids.size(), insertion_submaps.size());

    // 获取这两个submap中第一个的id
    const SubmapId matching_id = submap_ids.front();
    // 当前节点位姿在local坐标系下的T：T_track2local * R_track2gravity¯¹ = T_gravity2local or T_track2local
    const transform::Rigid2d local_pose_2d =
        transform::Project2D(constant_data->local_pose * // 当前节点在local坐标系下的节点位姿
                             transform::Rigid3d::Rotation(
                                 constant_data->gravity_alignment.inverse()));

    // 当前节点位姿在global坐标系下的T：
    // T_submap2global* T_submap2local¯¹ * T_gravity2local = T_local2global * T_gravity2local = T_gravity2global or T_track2global
    const transform::Rigid2d global_pose_2d =
        optimization_problem_->submap_data().at(matching_id).global_pose *
        constraints::ComputeSubmapPose(*insertion_submaps.front()).inverse() *
        local_pose_2d;
    
    // 把该节点的信息加入到OptimizationProblem中
    optimization_problem_->AddTrajectoryNode(
        matching_id.trajectory_id,
        optimization::NodeSpec2D{constant_data->time,
                                local_pose_2d,// T_gravity2local or T_track2local
                                global_pose_2d,// T_gravity2global or T_track2globaal
                                constant_data->gravity_alignment});// R_track2gravity

    // 遍历2个子图, 将节点加入子图的节点列表中, 计算子图原点与及节点间的约束(子图内约束)
    for (size_t i = 0; i < insertion_submaps.size(); ++i)
    {
      const SubmapId submap_id = submap_ids[i];
      // Even if this was the last node added to 'submap_id', the submap will
      // only be marked as finished in 'data_.submap_data' further below.
      CHECK(data_.submap_data.at(submap_id).state == SubmapState::kNoConstraintSearch);

      // 将node_id放到子图保存的node_ids的set中
      data_.submap_data.at(submap_id).node_ids.emplace(node_id);

      // notice: 子图内约束: tracking坐标系到子图坐标系的坐标变换(机器人水平状态下)
      // 当前节点位姿在各自submap坐标系下的T(子图内约束):T_track2submap
      // T_submap2local¯¹ * T_track2local = T_local2submap * T_track2local = T_track2submap or T_gravity2submap
      const transform::Rigid2d constraint_transform =
          constraints::ComputeSubmapPose(*insertion_submaps[i]).inverse() *
          local_pose_2d;

      // 新生成的 子图内约束 放入容器中
      data_.constraints.push_back(
          Constraint{submap_id,
                     node_id,
                     {transform::Embed3D(constraint_transform),// T_track2submap or T_gravity2submap
                      options_.matcher_translation_weight(),// 平移权重
                      options_.matcher_rotation_weight()},// 旋转权重
                     Constraint::INTRA_SUBMAP}); // 子图内约束标志
    } // end for

    // TODO(gaschler): Consider not searching for constraints against
    // trajectories scheduled for deletion.
    // TODO(danielsievers): Add a member variable and avoid having to copy
    // them out here.
    // 找到所有已经标记为kFinished状态的submap的id
    for (const auto& submap_id_data : data_.submap_data) 
    {
      // 如果是完成状态
      if (submap_id_data.data.state == SubmapState::kFinished) 
      {
        CHECK_EQ(submap_id_data.data.node_ids.count(node_id), 0);

        finished_submap_ids.emplace_back(submap_id_data.id);
      }
    }

    // 如果容器中的第一个submap是完成状态
    if (newly_finished_submap)
    {
      const SubmapId newly_finished_submap_id = submap_ids.front();
      InternalSubmapData& finished_submap_data =
          data_.submap_data.at(newly_finished_submap_id);

      // 检查它还是不是kNoConstraintSearch
      CHECK(finished_submap_data.state == SubmapState::kNoConstraintSearch);
      // 把该子图设置成kFinished
      finished_submap_data.state = SubmapState::kFinished;
      // 刚完成的这个子图里包含的所有节点id
      newly_finished_submap_node_ids = finished_submap_data.node_ids;
    }
  } // end {}

  // Step: notice: 当前节点与所有已经完成的子图进行约束的计算---实际上就是回环检测
  // 遍历所有完成状态下的子图
  for (const auto& submap_id : finished_submap_ids) 
  {
    // 计算所有旧的submap 和 当前新的节点间的约束
    // 当前节点与所有完成状态的子图进行匹配(子图间约束：一个当前节点 vs 所有旧子图)
    ComputeConstraint(node_id, submap_id);
  }
  

  // Step: 计算所有节点与刚完成子图间的约束---实际上就是回环检测
  // 如果容器中的第一个submap是完成状态
  if (newly_finished_submap)
  {
    // 我们有一个新的完整子图，所以我们考虑为旧节点添加约束。
    const SubmapId newly_finished_submap_id = submap_ids.front();

    // 遍历优化问题中的所有节点
    for (const auto& node_id_data : optimization_problem_->node_data())
    {
      const NodeId& node_id = node_id_data.id;

      // 如果当前节点不是新完成子图的内部节点(新完成子图内部的节点,不参与约束的计算)
      if (newly_finished_submap_node_ids.count(node_id) == 0) 
      {
        // 计算新的submap和旧的节点间的约束
        // 所有不属于新子图的节点与新子图进行匹配(子图间约束：所有旧节点 vs 一张新子图)
        ComputeConstraint(node_id, newly_finished_submap_id);
      }
      
    }
  }

  // 结束构建约束
  constraint_builder_.NotifyEndOfNode();

  absl::MutexLock locker(&mutex_);

  // 自从上次回环检测优化之后，节点个数
  ++num_nodes_since_last_loop_closure_;

  // Step: 插入的节点数大于optimize_every_n_nodes时执行一次优化
  // optimize_every_n_nodes = 0 时不进行优化, 这样就可以单独分析前端的效果
  if (options_.optimize_every_n_nodes() > 0 && // param: 多少个节点执行一次优化
      num_nodes_since_last_loop_closure_ > options_.optimize_every_n_nodes()) 
  {
    // notice: 正在建图时只有这一块会，当节点个数足够，执行优化
    return WorkItem::Result::kRunOptimization;
  }

  // 返回不优化
  return WorkItem::Result::kDoNotRunOptimization;
}

// 获取该 node 和该 submap 中的 node 中较新的时间
common::Time PoseGraph2D::GetLatestNodeTime(const NodeId& node_id,
                                            const SubmapId& submap_id) const 
{
  // 获取指定 id 节点的时间
  common::Time time = data_.trajectory_nodes.at(node_id).constant_data->time;
  // 获取指定 id 的 submap 的数据
  const InternalSubmapData& submap_data = data_.submap_data.at(submap_id);
  if (!submap_data.node_ids.empty()) {
    // 获取子图中的最后一个节点
    const NodeId last_submap_node_id =
        *data_.submap_data.at(submap_id).node_ids.rbegin();
    // 把时间更新为 节点建立时间 与 submap 中最后一个节点时间中 较晚的那个
    time = std::max(
        time,
        data_.trajectory_nodes.at(last_submap_node_id).constant_data->time);
  }
  return time;
}

// 根据新计算出的约束更新子图轨迹id与节点轨迹id的连接关系
void PoseGraph2D::UpdateTrajectoryConnectivity(const Constraint& constraint) {
  CHECK_EQ(constraint.tag, Constraint::INTER_SUBMAP);
  const common::Time time =
      GetLatestNodeTime(constraint.node_id, constraint.submap_id);
  data_.trajectory_connectivity_state.Connect(
      constraint.node_id.trajectory_id, constraint.submap_id.trajectory_id,
      time);
}

// 根据轨迹状态删除轨迹
void PoseGraph2D::DeleteTrajectoriesIfNeeded() {
  TrimmingHandle trimming_handle(this);
  for (auto& it : data_.trajectories_state) {
    if (it.second.deletion_state ==
        InternalTrajectoryState::DeletionState::WAIT_FOR_DELETION) {
      // TODO(gaschler): Consider directly deleting from data_, which may be
      // more complete.
      auto submap_ids = trimming_handle.GetSubmapIds(it.first);
      for (auto& submap_id : submap_ids) {
        trimming_handle.TrimSubmap(submap_id);
      }
      it.second.state = TrajectoryState::DELETED;
      it.second.deletion_state = InternalTrajectoryState::DeletionState::NORMAL;
    }
  }
}

// 将计算完的约束结果进行保存, 并执行优化
void PoseGraph2D::HandleWorkQueue(
    const constraints::ConstraintBuilder2D::Result& result) {
  {
    absl::MutexLock locker(&mutex_);
    // Step: 把新计算出的约束信息添加到data_.constraints向量的末尾处
    data_.constraints.insert(data_.constraints.end(), result.begin(),
                             result.end());
  }

  // Step: 执行优化
  RunOptimization();

  // Step: 如果已经设置了全局优化的回调函数, 将数据传入回调函数
  if (global_slam_optimization_callback_) {
    std::map<int, NodeId> trajectory_id_to_last_optimized_node_id;
    std::map<int, SubmapId> trajectory_id_to_last_optimized_submap_id;
    {
      absl::MutexLock locker(&mutex_);
      const auto& submap_data = optimization_problem_->submap_data();
      const auto& node_data = optimization_problem_->node_data();
      for (const int trajectory_id : node_data.trajectory_ids()) {
        if (node_data.SizeOfTrajectoryOrZero(trajectory_id) == 0 ||
            submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0) {
          continue;
        }
        // 计算约束后的最后一个节点与submap的id
        trajectory_id_to_last_optimized_node_id.emplace(
            trajectory_id,
            std::prev(node_data.EndOfTrajectory(trajectory_id))->id);
        trajectory_id_to_last_optimized_submap_id.emplace(
            trajectory_id,
            std::prev(submap_data.EndOfTrajectory(trajectory_id))->id);
      }
    }
    // 将优化后最后一个节点的submap id 与 节点id 传入回调函数进行处理
    global_slam_optimization_callback_(
        trajectory_id_to_last_optimized_submap_id,
        trajectory_id_to_last_optimized_node_id);
  }

  {
    absl::MutexLock locker(&mutex_);
    // 根据优化后的约束更新子图轨迹id与节点轨迹id的连接关系
    for (const Constraint& constraint : result) {
      UpdateTrajectoryConnectivity(constraint);
    }

    // 根据轨迹状态删除轨迹
    DeleteTrajectoriesIfNeeded();
    
    TrimmingHandle trimming_handle(this);
    // 进行子图的裁剪, 如果没有裁剪器就不裁剪
    for (auto& trimmer : trimmers_) {
      trimmer->Trim(&trimming_handle); // PureLocalizationTrimmer::Trim()
    }
    // 如果裁剪器处于完成状态, 就把裁剪器删除掉
    trimmers_.erase(
        // c++11: std::remove_if 如果回调函数函数返回真,则将当前所指向的参数移到尾部,返回值是被移动区域的首个元素
        std::remove_if(trimmers_.begin(), trimmers_.end(),
                       [](std::unique_ptr<PoseGraphTrimmer>& trimmer) {
                         return trimmer->IsFinished(); // 调用PureLocalizationTrimmer::IsFinished()
                       }),
        trimmers_.end());

    // 执行完优化后，把这个变量重新置为0，下一次满足指定个数时再执行优化
    num_nodes_since_last_loop_closure_ = 0;

    // Update the gauges that count the current number of constraints.
    // 计算相同轨迹下与不同轨迹下 inter_constraints 的数量, 放入Metric中
    double inter_constraints_same_trajectory = 0;
    double inter_constraints_different_trajectory = 0;
    for (const auto& constraint : data_.constraints) {
      if (constraint.tag ==
          cartographer::mapping::PoseGraph::Constraint::INTRA_SUBMAP) {
        continue;
      }
      if (constraint.node_id.trajectory_id ==
          constraint.submap_id.trajectory_id) {
        ++inter_constraints_same_trajectory;
      } else {
        ++inter_constraints_different_trajectory;
      }
    }
    kConstraintsSameTrajectoryMetric->Set(inter_constraints_same_trajectory);
    kConstraintsDifferentTrajectoryMetric->Set(
        inter_constraints_different_trajectory);

  } // end {}

  // Step: 优化执行结束了, 继续处理任务队列中的其他任务
  DrainWorkQueue();
}

// 在调用线程上执行工作队列中的待处理任务, 直到队列为空或需要优化时退出循环
// 在线程池中执行
void PoseGraph2D::DrainWorkQueue() 
{
  bool process_work_queue = true;
  size_t work_queue_size;

  // 循环执行, 直到队列为空或需要优化时退出循环
  while (process_work_queue) 
  {
    std::function<WorkItem::Result()> work_item;

    {
      absl::MutexLock locker(&work_queue_mutex_);
      // 退出条件1 如果任务队列空了, 就将work_queue_的指针删除
      if (work_queue_->empty()) 
      {
        work_queue_.reset();
        return;
      }
      // 取出第一个任务(lambda表达式)，但是没有执行
      work_item = work_queue_->front().task;

      // 将取出的第一个任务从任务队列中删掉
      work_queue_->pop_front();
      work_queue_size = work_queue_->size();
      kWorkQueueSizeMetric->Set(work_queue_size);
    }
    /*
    notice: 开始执行任务(lambda表达式)：work_queue_->front()()，返回值为Result
    std::function<Result()> task; task为 一个函数的名字, 这个函数返回值类型为Result(枚举类型enum class Result)
    退出条件2 若lambda返回的结果是:优化, process_work_queue==false，退出循环，若返回的结果为优化则继续循环
    */
    process_work_queue = work_item() == WorkItem::Result::kDoNotRunOptimization;
  }
  
  LOG(INFO) << "Remaining work items in queue: " << work_queue_size;
  // We have to optimize again.
  // 退出循环后, 首先等待计算约束中的任务执行完, 再执行HandleWorkQueue,进行优化
  constraint_builder_.WhenDone(
      [this](const constraints::ConstraintBuilder2D::Result& result) 
      {
        HandleWorkQueue(result);
      });
}

// 等待所有的计算任务执行完成
void PoseGraph2D::WaitForAllComputations() {
  int num_trajectory_nodes;
  {
    absl::MutexLock locker(&mutex_);
    num_trajectory_nodes = data_.num_trajectory_nodes;
  }

  const int num_finished_nodes_at_start =
      constraint_builder_.GetNumFinishedNodes();

  // 报告节点计算的进度
  auto report_progress = [this, num_trajectory_nodes,
                          num_finished_nodes_at_start]() {
    // Log progress on nodes only when we are actually processing nodes.
    if (num_trajectory_nodes != num_finished_nodes_at_start) {
      std::ostringstream progress_info;
      progress_info << "Optimizing: " << std::fixed << std::setprecision(1)
                    << 100. *
                           (constraint_builder_.GetNumFinishedNodes() -
                            num_finished_nodes_at_start) /
                           (num_trajectory_nodes - num_finished_nodes_at_start)
                    << "%...";
      std::cout << "\r\x1b[K" << progress_info.str() << std::flush;
    }
  };

  // First wait for the work queue to drain so that it's safe to schedule
  // a WhenDone() callback.
  {
    const auto predicate = [this]()
                               EXCLUSIVE_LOCKS_REQUIRED(work_queue_mutex_) {
                                 return work_queue_ == nullptr;
                               };
    absl::MutexLock locker(&work_queue_mutex_);
    // 等待工作队列为空
    while (!work_queue_mutex_.AwaitWithTimeout(
        absl::Condition(&predicate),
        absl::FromChrono(common::FromSeconds(1.)))) { // 1秒打印一次进度
      report_progress();
    }
  }

  // Now wait for any pending constraint computations to finish.
  // 现在等待任何挂起的约束计算完成
  absl::MutexLock locker(&mutex_);
  bool notification = false;
  constraint_builder_.WhenDone(
      [this,
       &notification](const constraints::ConstraintBuilder2D::Result& result)
          LOCKS_EXCLUDED(mutex_) {
            absl::MutexLock locker(&mutex_);
            // 保存新计算的约束
            data_.constraints.insert(data_.constraints.end(), result.begin(),
                                     result.end());
            notification = true;
          });

  const auto predicate = [&notification]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
    return notification;
  };

  // 等待直到notification为true
  while (!mutex_.AwaitWithTimeout(absl::Condition(&predicate),
                                  absl::FromChrono(common::FromSeconds(1.)))) {
    report_progress();
  }
  CHECK_EQ(constraint_builder_.GetNumFinishedNodes(), num_trajectory_nodes);
  std::cout << "\r\x1b[KOptimizing: Done.     " << std::endl;
}

void PoseGraph2D::DeleteTrajectory(const int trajectory_id) {
  {
    absl::MutexLock locker(&mutex_);
    auto it = data_.trajectories_state.find(trajectory_id);
    if (it == data_.trajectories_state.end()) {
      LOG(WARNING) << "Skipping request to delete non-existing trajectory_id: "
                   << trajectory_id;
      return;
    }
    it->second.deletion_state =
        InternalTrajectoryState::DeletionState::SCHEDULED_FOR_DELETION;
  }
  AddWorkItem([this, trajectory_id]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    CHECK(data_.trajectories_state.at(trajectory_id).state !=
          TrajectoryState::ACTIVE);
    CHECK(data_.trajectories_state.at(trajectory_id).state !=
          TrajectoryState::DELETED);
    CHECK(data_.trajectories_state.at(trajectory_id).deletion_state ==
          InternalTrajectoryState::DeletionState::SCHEDULED_FOR_DELETION);
    data_.trajectories_state.at(trajectory_id).deletion_state =
        InternalTrajectoryState::DeletionState::WAIT_FOR_DELETION;
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

// 结束指定id的轨迹
void PoseGraph2D::FinishTrajectory(const int trajectory_id) {
  AddWorkItem([this, trajectory_id]() LOCKS_EXCLUDED(mutex_) 
  {
    absl::MutexLock locker(&mutex_);
    CHECK(!IsTrajectoryFinished(trajectory_id));
    // 把轨迹的状态设置为FINISHED
    data_.trajectories_state[trajectory_id].state = TrajectoryState::FINISHED;
    // 把轨迹内的所有子图的状态设置为kFinished
    for (const auto& submap : data_.submap_data.trajectory(trajectory_id)) {
      data_.submap_data.at(submap.id).state = SubmapState::kFinished;
    }
    // notice: 轨迹结束后执行一次优化
    return WorkItem::Result::kRunOptimization;
  });
}

// 判断轨迹是否是完成状态
bool PoseGraph2D::IsTrajectoryFinished(const int trajectory_id) const {
  return data_.trajectories_state.count(trajectory_id) != 0 &&
         data_.trajectories_state.at(trajectory_id).state ==
             TrajectoryState::FINISHED;
}

// 将指定轨迹id设置为FROZEN状态
void PoseGraph2D::FreezeTrajectory(const int trajectory_id) {
  {
    absl::MutexLock locker(&mutex_);
    // 与自己做连接即标记为连接关系的尽头
    data_.trajectory_connectivity_state.Add(trajectory_id);
  }
  AddWorkItem([this, trajectory_id]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    CHECK(!IsTrajectoryFrozen(trajectory_id));
    // Connect multiple frozen trajectories among each other.
    // This is required for localization against multiple frozen trajectories
    // because we lose inter-trajectory constraints when freezing.
    for (const auto& entry : data_.trajectories_state) {
      const int other_trajectory_id = entry.first;
      if (!IsTrajectoryFrozen(other_trajectory_id)) {
        continue;
      }
      if (data_.trajectory_connectivity_state.TransitivelyConnected(
              trajectory_id, other_trajectory_id)) {
        // Already connected, nothing to do.
        continue;
      }
      data_.trajectory_connectivity_state.Connect(
          trajectory_id, other_trajectory_id, common::FromUniversal(0));
    }
    data_.trajectories_state[trajectory_id].state = TrajectoryState::FROZEN;
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

// 判断轨迹是否是 FROZEN 状态
bool PoseGraph2D::IsTrajectoryFrozen(const int trajectory_id) const {
  return data_.trajectories_state.count(trajectory_id) != 0 &&
         data_.trajectories_state.at(trajectory_id).state ==
             TrajectoryState::FROZEN;
}

// 从proto流数据中添加Submap
void PoseGraph2D::AddSubmapFromProto(
    const transform::Rigid3d& global_submap_pose, const proto::Submap& submap) {
  if (!submap.has_submap_2d()) {
    return;
  }

  const SubmapId submap_id = {submap.submap_id().trajectory_id(),
                              submap.submap_id().submap_index()};

  const transform::Rigid2d global_submap_pose_2d =
      transform::Project2D(global_submap_pose);
  {
    absl::MutexLock locker(&mutex_);
    const std::shared_ptr<const Submap2D> submap_ptr =
        std::make_shared<const Submap2D>(submap.submap_2d(),
                                         &conversion_tables_);
    AddTrajectoryIfNeeded(submap_id.trajectory_id);
    if (!CanAddWorkItemModifying(submap_id.trajectory_id)) return;
    data_.submap_data.Insert(submap_id, InternalSubmapData());
    data_.submap_data.at(submap_id).submap = submap_ptr;
    // Immediately show the submap at the 'global_submap_pose'.
    data_.global_submap_poses_2d.Insert(
        submap_id, optimization::SubmapSpec2D{global_submap_pose_2d});
  }

  // TODO(MichaelGrupp): MapBuilder does freezing before deserializing submaps,
  // so this should be fine.
  if (IsTrajectoryFrozen(submap_id.trajectory_id)) {
    kFrozenSubmapsMetric->Increment();
  } else {
    kActiveSubmapsMetric->Increment();
  }

  AddWorkItem(
      [this, submap_id, global_submap_pose_2d]() LOCKS_EXCLUDED(mutex_) {
        absl::MutexLock locker(&mutex_);
        data_.submap_data.at(submap_id).state = SubmapState::kFinished;
        optimization_problem_->InsertSubmap(submap_id, global_submap_pose_2d);
        return WorkItem::Result::kDoNotRunOptimization;
      });
}

// 从proto流数据中添加节点
void PoseGraph2D::AddNodeFromProto(const transform::Rigid3d& global_pose,
                                   const proto::Node& node) {
  const NodeId node_id = {node.node_id().trajectory_id(),
                          node.node_id().node_index()};
  std::shared_ptr<const TrajectoryNode::Data> constant_data =
      std::make_shared<const TrajectoryNode::Data>(FromProto(node.node_data()));

  {
    absl::MutexLock locker(&mutex_);
    AddTrajectoryIfNeeded(node_id.trajectory_id);
    if (!CanAddWorkItemModifying(node_id.trajectory_id)) return;
    data_.trajectory_nodes.Insert(node_id,
                                  TrajectoryNode{constant_data, global_pose});
  }

  AddWorkItem([this, node_id, global_pose]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    const auto& constant_data =
        data_.trajectory_nodes.at(node_id).constant_data;
    const auto gravity_alignment_inverse = transform::Rigid3d::Rotation(
        constant_data->gravity_alignment.inverse());
    optimization_problem_->InsertTrajectoryNode(
        node_id,
        optimization::NodeSpec2D{
            constant_data->time,
            transform::Project2D(constant_data->local_pose *
                                 gravity_alignment_inverse),
            transform::Project2D(global_pose * gravity_alignment_inverse),
            constant_data->gravity_alignment});
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

// 从proto流数据中添加节点信息
void PoseGraph2D::SetTrajectoryDataFromProto(
    const proto::TrajectoryData& data) {
  TrajectoryData trajectory_data;
  // gravity_constant and imu_calibration are omitted as its not used in 2d

  if (data.has_fixed_frame_origin_in_map()) {
    trajectory_data.fixed_frame_origin_in_map =
        transform::ToRigid3(data.fixed_frame_origin_in_map());

    const int trajectory_id = data.trajectory_id();
    AddWorkItem([this, trajectory_id, trajectory_data]()
                    LOCKS_EXCLUDED(mutex_) {
                      absl::MutexLock locker(&mutex_);
                      if (CanAddWorkItemModifying(trajectory_id)) {
                        optimization_problem_->SetTrajectoryData(
                            trajectory_id, trajectory_data);
                      }
                      return WorkItem::Result::kDoNotRunOptimization;
                    });
  }
}

// 将节点添加到submap中的节点列表中
void PoseGraph2D::AddNodeToSubmap(const NodeId& node_id,
                                  const SubmapId& submap_id) {
  AddWorkItem([this, node_id, submap_id]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    if (CanAddWorkItemModifying(submap_id.trajectory_id)) {
      data_.submap_data.at(submap_id).node_ids.insert(node_id);
    }
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

// 向data_中添加约束数据
void PoseGraph2D::AddSerializedConstraints(
    const std::vector<Constraint>& constraints) {
  AddWorkItem([this, constraints]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    for (const auto& constraint : constraints) {
      CHECK(data_.trajectory_nodes.Contains(constraint.node_id));
      CHECK(data_.submap_data.Contains(constraint.submap_id));
      CHECK(data_.trajectory_nodes.at(constraint.node_id).constant_data !=
            nullptr);
      CHECK(data_.submap_data.at(constraint.submap_id).submap != nullptr);
      switch (constraint.tag) {
        case Constraint::Tag::INTRA_SUBMAP:
          CHECK(data_.submap_data.at(constraint.submap_id)
                    .node_ids.emplace(constraint.node_id)
                    .second);
          break;
        case Constraint::Tag::INTER_SUBMAP:
          UpdateTrajectoryConnectivity(constraint);
          break;
      }
      const Constraint::Pose pose = {
          constraint.pose.zbar_ij *
              transform::Rigid3d::Rotation(
                  data_.trajectory_nodes.at(constraint.node_id)
                      .constant_data->gravity_alignment.inverse()),
          constraint.pose.translation_weight, constraint.pose.rotation_weight};
      data_.constraints.push_back(Constraint{
          constraint.submap_id, constraint.node_id, pose, constraint.tag});
    }
    LOG(INFO) << "Loaded " << constraints.size() << " constraints.";
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

// map_builder.cc中调用, 纯定位时添加PureLocalizationTrimmer
void PoseGraph2D::AddTrimmer(std::unique_ptr<PoseGraphTrimmer> trimmer) {
  // C++11 does not allow us to move a unique_ptr into a lambda.
  PoseGraphTrimmer* const trimmer_ptr = trimmer.release();
  AddWorkItem([this, trimmer_ptr]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    trimmers_.emplace_back(trimmer_ptr);
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

// notice: 执行最后一次的优化, 等待所有的计算任务结束
void PoseGraph2D::RunFinalOptimization() 
{
  {
    AddWorkItem([this]() LOCKS_EXCLUDED(mutex_) 
    {
      absl::MutexLock locker(&mutex_);
      // 设置更多的最大迭代次数
      optimization_problem_->SetMaxNumIterations(
          options_.max_num_final_iterations());
      // 执行一次优化
      return WorkItem::Result::kRunOptimization;
    });
    AddWorkItem([this]() LOCKS_EXCLUDED(mutex_) 
    {
      absl::MutexLock locker(&mutex_);
      // 再设置回去
      optimization_problem_->SetMaxNumIterations(
          options_.optimization_problem_options()
              .ceres_solver_options()
              .max_num_iterations());
      // 结束了不用执行优化
      return WorkItem::Result::kDoNotRunOptimization;
    });
  }
  WaitForAllComputations();
}

// 进行优化处理, 并使用优化结果对保存的数据进行更新
void PoseGraph2D::RunOptimization() {
  // 如果submap为空直接退出
  if (optimization_problem_->submap_data().empty()) {
    return;
  }

  // No other thread is accessing the optimization_problem_,
  // data_.constraints, data_.frozen_trajectories and data_.landmark_nodes
  // when executing the Solve. Solve is time consuming, so not taking the mutex
  // before Solve to avoid blocking foreground processing.
  // Solve 比较耗时, 所以在执行 Solve 之前不要加互斥锁, 以免阻塞其他的任务处理
  // landmark直接参与优化问题
  optimization_problem_->Solve(data_.constraints, GetTrajectoryStates(),
                               data_.landmark_nodes);

  absl::MutexLock locker(&mutex_);

  // 获取优化后的结果
  // submap_data的类型是 MapById<SubmapId, optimization::SubmapSpec2D> 
  const auto& submap_data = optimization_problem_->submap_data();

  // node_data的类型是 MapById<NodeId, NodeSpec2D>
  // node_data是优化后的所有节点的新位姿
  const auto& node_data = optimization_problem_->node_data();

  // 更新轨迹内的节点位置
  for (const int trajectory_id : node_data.trajectory_ids()) {

    // Step: 根据优化后的结果对data_.trajectory_nodes的global_pose进行更新
    for (const auto& node : node_data.trajectory(trajectory_id)) {
      // node 是 IdDataReference 类型
      // mutable_trajectory_node是TrajectoryNode类型
      auto& mutable_trajectory_node = data_.trajectory_nodes.at(node.id);
      // 将优化后的二维节点位姿旋转到机器人的姿态上得到global_pose
      mutable_trajectory_node.global_pose =
          transform::Embed3D(node.data.global_pose_2d) * 
          transform::Rigid3d::Rotation(
              mutable_trajectory_node.constant_data->gravity_alignment);
    }

    // Extrapolate all point cloud poses that were not included in the
    // 'optimization_problem_' yet.
    // 推断尚未包含在“optimization_problem_”中的所有点云姿势

    // 根据submap_data最后一个被优化的位姿, 计算global坐标系指向local坐标系的坐标变换
    const auto local_to_new_global =
        ComputeLocalToGlobalTransform(submap_data, trajectory_id);
    // 优化前的 global坐标系指向local坐标系的坐标变换
    const auto local_to_old_global = ComputeLocalToGlobalTransform(
        data_.global_submap_poses_2d, trajectory_id);
    // 优化产生的改变量
    const transform::Rigid3d old_global_to_new_global =
        local_to_new_global * local_to_old_global.inverse();
    // 这一次优化的node的最后一个id
    const NodeId last_optimized_node_id =
        std::prev(node_data.EndOfTrajectory(trajectory_id))->id;
    // 指向下一个没有优化的节点
    auto node_it =
        std::next(data_.trajectory_nodes.find(last_optimized_node_id));

    // Step: 根据之前的位姿改变量, 对没有优化过的位姿进行校正
    for (; node_it != data_.trajectory_nodes.EndOfTrajectory(trajectory_id);
         ++node_it) {
      auto& mutable_trajectory_node = data_.trajectory_nodes.at(node_it->id);
      mutable_trajectory_node.global_pose =
          old_global_to_new_global * mutable_trajectory_node.global_pose;
    }

  } // end for trajectory_id

  // 更新data_.landmark_nodes
  for (const auto& landmark : optimization_problem_->landmark_data()) {
    data_.landmark_nodes[landmark.first].global_landmark_pose = landmark.second;
  }

  // 更新所有submap的位姿
  data_.global_submap_poses_2d = submap_data;
}

// 根据轨迹状态判断是否可以添加任务
bool PoseGraph2D::CanAddWorkItemModifying(int trajectory_id) {
  auto it = data_.trajectories_state.find(trajectory_id);
  if (it == data_.trajectories_state.end()) {
    return true;
  }
  if (it->second.state == TrajectoryState::FINISHED) {
    // TODO(gaschler): Replace all FATAL to WARNING after some testing.
    LOG(FATAL) << "trajectory_id " << trajectory_id
               << " has finished "
                  "but modification is requested, skipping.";
    return false;
  }
  if (it->second.deletion_state !=
      InternalTrajectoryState::DeletionState::NORMAL) {
    LOG(FATAL) << "trajectory_id " << trajectory_id
               << " has been scheduled for deletion "
                  "but modification is requested, skipping.";
    return false;
  }
  if (it->second.state == TrajectoryState::DELETED) {
    LOG(FATAL) << "trajectory_id " << trajectory_id
               << " has been deleted "
                  "but modification is requested, skipping.";
    return false;
  }
  return true;
}

// 获取所有的节点的信息
MapById<NodeId, TrajectoryNode> PoseGraph2D::GetTrajectoryNodes() const {
  absl::MutexLock locker(&mutex_);
  return data_.trajectory_nodes;
}

// 获取所有的轨迹节点的id与位姿
MapById<NodeId, TrajectoryNodePose> PoseGraph2D::GetTrajectoryNodePoses()
    const {
  MapById<NodeId, TrajectoryNodePose> node_poses;
  absl::MutexLock locker(&mutex_);
  // trajectory_nodes是 nodeid + TrajectoryNode
  for (const auto& node_id_data : data_.trajectory_nodes) {
    absl::optional<TrajectoryNodePose::ConstantPoseData> constant_pose_data;
    if (node_id_data.data.constant_data != nullptr) {
      constant_pose_data = TrajectoryNodePose::ConstantPoseData{
          node_id_data.data.constant_data->time,
          node_id_data.data.constant_data->local_pose};
    }
    node_poses.Insert(
        node_id_data.id,
        TrajectoryNodePose{node_id_data.data.global_pose, constant_pose_data});
  }
  return node_poses;
}

// 返回图结构中的所有的轨迹状态
std::map<int, PoseGraphInterface::TrajectoryState>
PoseGraph2D::GetTrajectoryStates() const {
  std::map<int, PoseGraphInterface::TrajectoryState> trajectories_state;
  absl::MutexLock locker(&mutex_);
  for (const auto& it : data_.trajectories_state) {
    trajectories_state[it.first] = it.second.state;
  }
  return trajectories_state;
}

// 获取所有的landmark的位姿
std::map<std::string, transform::Rigid3d> PoseGraph2D::GetLandmarkPoses()
    const {
  std::map<std::string, transform::Rigid3d> landmark_poses;
  absl::MutexLock locker(&mutex_);
  for (const auto& landmark : data_.landmark_nodes) {
    // Landmark without value has not been optimized yet.
    if (!landmark.second.global_landmark_pose.has_value()) continue;
    landmark_poses[landmark.first] =
        landmark.second.global_landmark_pose.value();
  }
  return landmark_poses;
}

// 设置landmark在global坐标系下的坐标, 只有在从proto加载状态时进行使用
void PoseGraph2D::SetLandmarkPose(const std::string& landmark_id,
                                  const transform::Rigid3d& global_pose,
                                  const bool frozen) {
  AddWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    data_.landmark_nodes[landmark_id].global_landmark_pose = global_pose;
    data_.landmark_nodes[landmark_id].frozen = frozen;
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

// 获取优化问题中的imu数据, 会返回空值, 因为2D优化中不使用IMU数据
sensor::MapByTime<sensor::ImuData> PoseGraph2D::GetImuData() const {
  absl::MutexLock locker(&mutex_);
  return optimization_problem_->imu_data();
}

// 获取优化问题中的里程计数据
sensor::MapByTime<sensor::OdometryData> PoseGraph2D::GetOdometryData() const {
  absl::MutexLock locker(&mutex_);
  return optimization_problem_->odometry_data();
}

// 获取所有的landmark_nodes
std::map<std::string /* landmark ID */, PoseGraphInterface::LandmarkNode>
PoseGraph2D::GetLandmarkNodes() const {
  absl::MutexLock locker(&mutex_);
  return data_.landmark_nodes;
}

std::map<int, PoseGraphInterface::TrajectoryData>
PoseGraph2D::GetTrajectoryData() const {
  absl::MutexLock locker(&mutex_);
  return optimization_problem_->trajectory_data();
}

// 获取优化问题中的gps数据
sensor::MapByTime<sensor::FixedFramePoseData>
PoseGraph2D::GetFixedFramePoseData() const {
  absl::MutexLock locker(&mutex_);
  return optimization_problem_->fixed_frame_pose_data();
}

// 返回位姿图结构中的所有的约束
std::vector<PoseGraphInterface::Constraint> PoseGraph2D::constraints() const {
  std::vector<PoseGraphInterface::Constraint> result;
  absl::MutexLock locker(&mutex_);
  for (const Constraint& constraint : data_.constraints) {
    result.push_back(Constraint{
        constraint.submap_id, constraint.node_id,
        Constraint::Pose{constraint.pose.zbar_ij *
                             transform::Rigid3d::Rotation(
                                 data_.trajectory_nodes.at(constraint.node_id)
                                     .constant_data->gravity_alignment),
                         constraint.pose.translation_weight,
                         constraint.pose.rotation_weight},
        constraint.tag});
  }
  return result;
}

/**
 * @brief 设置当前轨迹的起始坐标
 * 
 * @param[in] from_trajectory_id 当前的轨迹id
 * @param[in] to_trajectory_id pose相对于的轨迹的id
 * @param[in] pose 在to_trajectory_id中的坐标
 * @param[in] time 当前的时间
 */
void PoseGraph2D::SetInitialTrajectoryPose(const int from_trajectory_id,
                                           const int to_trajectory_id,
                                           const transform::Rigid3d& pose,
                                           const common::Time time) {
  absl::MutexLock locker(&mutex_);
  data_.initial_trajectory_poses[from_trajectory_id] =
      InitialTrajectoryPose{to_trajectory_id, pose, time};
}

// 线性插值计算指定时间的global_pose
transform::Rigid3d PoseGraph2D::GetInterpolatedGlobalTrajectoryPose(
    const int trajectory_id, const common::Time time) const {
  CHECK_GT(data_.trajectory_nodes.SizeOfTrajectoryOrZero(trajectory_id), 0);
  const auto it = data_.trajectory_nodes.lower_bound(trajectory_id, time);
  if (it == data_.trajectory_nodes.BeginOfTrajectory(trajectory_id)) {
    return data_.trajectory_nodes.BeginOfTrajectory(trajectory_id)
        ->data.global_pose;
  }
  if (it == data_.trajectory_nodes.EndOfTrajectory(trajectory_id)) {
    return std::prev(data_.trajectory_nodes.EndOfTrajectory(trajectory_id))
        ->data.global_pose;
  }
  // 线性插值计算指定时间的global_pose
  return transform::Interpolate(
             transform::TimestampedTransform{std::prev(it)->data.time(),
                                             std::prev(it)->data.global_pose},
             transform::TimestampedTransform{it->data.time(),
                                             it->data.global_pose},
             time)
      .transform;
}

// 计算 T_local2global
transform::Rigid3d PoseGraph2D::GetLocalToGlobalTransform(
    const int trajectory_id) const 
{
  // 可能同时间有多个线程调用这同一个函数(data_数据), 所以要加锁
  absl::MutexLock locker(&mutex_);
  // data_一开始进来为空
  return ComputeLocalToGlobalTransform(data_.global_submap_poses_2d,
                                       trajectory_id);
}

// 获取链接关系中的轨迹ID的列表
std::vector<std::vector<int>> PoseGraph2D::GetConnectedTrajectories() const {
  absl::MutexLock locker(&mutex_);
  return data_.trajectory_connectivity_state.Components();
}

// 获取指定id的submap地图
PoseGraphInterface::SubmapData PoseGraph2D::GetSubmapData(
    const SubmapId& submap_id) const 
{
  absl::MutexLock locker(&mutex_);
  return GetSubmapDataUnderLock(submap_id);
}

// 获取所有的submap地图
MapById<SubmapId, PoseGraphInterface::SubmapData>
PoseGraph2D::GetAllSubmapData() const {
  absl::MutexLock locker(&mutex_);
  return GetSubmapDataUnderLock();
}

// 获取所有的submap的原点的坐标
MapById<SubmapId, PoseGraphInterface::SubmapPose>
PoseGraph2D::GetAllSubmapPoses() const {
  absl::MutexLock locker(&mutex_);
  MapById<SubmapId, SubmapPose> submap_poses;
  for (const auto& submap_id_data : data_.submap_data) {
    auto submap_data = GetSubmapDataUnderLock(submap_id_data.id);
    submap_poses.Insert(
        submap_id_data.id,
        PoseGraph::SubmapPose{submap_data.submap->num_range_data(),
                              submap_data.pose});
  }
  return submap_poses;
}

// 计算 T_local2global
transform::Rigid3d PoseGraph2D::ComputeLocalToGlobalTransform(
    const MapById<SubmapId, optimization::SubmapSpec2D>& global_submap_poses,
    const int trajectory_id) const 
{
  // global坐标系下的起始位姿
  auto begin_it = global_submap_poses.BeginOfTrajectory(trajectory_id);
  auto end_it = global_submap_poses.EndOfTrajectory(trajectory_id);

  // 没找到这个轨迹id 指针为空
  if (begin_it == end_it)
  {
    const auto it = data_.initial_trajectory_poses.find(trajectory_id);
    // 如果设置了后端优化的初始位姿
    if (it != data_.initial_trajectory_poses.end()) 
    {
      // 获取初始位姿
      return GetInterpolatedGlobalTrajectoryPose(it->second.to_trajectory_id,
                                                 it->second.time) *
             it->second.relative_pose;
    }
    // notice: 如果没设置后端优化的初始位姿
    else 
    {
      // 返回：T_local2global = 单位阵
      return transform::Rigid3d::Identity();
    }
  }

  // 找到了就获取优化后的最后一个子图的id
  const SubmapId last_optimized_submap_id = std::prev(end_it)->id;

  // Pose_submap2global * Pose_submap2local¯¹ = Pose_submap2global * Pose_local2submap = Pose_local2global
  return transform::Embed3D(
             global_submap_poses.at(last_optimized_submap_id).global_pose) *
         data_.submap_data.at(last_optimized_submap_id)
             .submap->local_pose()
             .inverse();
}

PoseGraphInterface::SubmapData PoseGraph2D::GetSubmapDataUnderLock(
    const SubmapId& submap_id) const {
  const auto it = data_.submap_data.find(submap_id);
  if (it == data_.submap_data.end()) {
    return {};
  }
  auto submap = it->data.submap;
  if (data_.global_submap_poses_2d.Contains(submap_id)) {
    // We already have an optimized pose.
    return {submap,
            transform::Embed3D(
                data_.global_submap_poses_2d.at(submap_id).global_pose)};
  }
  // We have to extrapolate.
  return {submap, ComputeLocalToGlobalTransform(data_.global_submap_poses_2d,
                                                submap_id.trajectory_id) *
                      submap->local_pose()};
}


/************ PoseGraph2D::TrimmingHandle ***********************/

PoseGraph2D::TrimmingHandle::TrimmingHandle(PoseGraph2D* const parent)
    : parent_(parent) {}

int PoseGraph2D::TrimmingHandle::num_submaps(const int trajectory_id) const {
  const auto& submap_data = parent_->optimization_problem_->submap_data();
  return submap_data.SizeOfTrajectoryOrZero(trajectory_id);
}

MapById<SubmapId, PoseGraphInterface::SubmapData>
PoseGraph2D::TrimmingHandle::GetOptimizedSubmapData() const {
  MapById<SubmapId, PoseGraphInterface::SubmapData> submaps;
  for (const auto& submap_id_data : parent_->data_.submap_data) {
    if (submap_id_data.data.state != SubmapState::kFinished ||
        !parent_->data_.global_submap_poses_2d.Contains(submap_id_data.id)) {
      continue;
    }
    submaps.Insert(
        submap_id_data.id,
        SubmapData{submap_id_data.data.submap,
                   transform::Embed3D(parent_->data_.global_submap_poses_2d
                                          .at(submap_id_data.id)
                                          .global_pose)});
  }
  return submaps;
}

// 获取所有的SubmapId
std::vector<SubmapId> PoseGraph2D::TrimmingHandle::GetSubmapIds(
    int trajectory_id) const {
  std::vector<SubmapId> submap_ids;
  const auto& submap_data = parent_->optimization_problem_->submap_data();
  for (const auto& it : submap_data.trajectory(trajectory_id)) {
    submap_ids.push_back(it.id);
  }
  return submap_ids;
}

const MapById<NodeId, TrajectoryNode>&
PoseGraph2D::TrimmingHandle::GetTrajectoryNodes() const {
  return parent_->data_.trajectory_nodes;
}

const std::vector<PoseGraphInterface::Constraint>&
PoseGraph2D::TrimmingHandle::GetConstraints() const {
  return parent_->data_.constraints;
}

// 轨迹结束了, 裁剪器就结束
bool PoseGraph2D::TrimmingHandle::IsFinished(const int trajectory_id) const {
  return parent_->IsTrajectoryFinished(trajectory_id);
}

void PoseGraph2D::TrimmingHandle::SetTrajectoryState(int trajectory_id,
                                                     TrajectoryState state) {
  parent_->data_.trajectories_state[trajectory_id].state = state;
}

// 删除指定id的子图, 并删除相关的约束,匹配器,与节点
void PoseGraph2D::TrimmingHandle::TrimSubmap(const SubmapId& submap_id) {
  // TODO(hrapp): We have to make sure that the trajectory has been finished
  // if we want to delete the last submaps.
  // 只有kFinished状态的子图才能够裁剪
  CHECK(parent_->data_.submap_data.at(submap_id).state ==
        SubmapState::kFinished);

  // Compile all nodes that are still INTRA_SUBMAP constrained to other submaps
  // once the submap with 'submap_id' is gone.
  // We need to use node_ids instead of constraints here to be also compatible
  // with frozen trajectories that don't have intra-constraints.
  // 获取除submap_id外的所有子图的所有节点的id
  std::set<NodeId> nodes_to_retain;
  for (const auto& submap_data : parent_->data_.submap_data) {
    if (submap_data.id != submap_id) {
      nodes_to_retain.insert(submap_data.data.node_ids.begin(),
                             submap_data.data.node_ids.end());
    }
  }

  // Remove all nodes that are exlusively associated to 'submap_id'.
  // 找到在submap_id的子图内部同时不在别的子图内的节点, 这些节点需要删除
  std::set<NodeId> nodes_to_remove;
  // c++11: std::set_difference 求set的差集, 在first_set中出现, 在second_set中不出现的元素
  std::set_difference(parent_->data_.submap_data.at(submap_id).node_ids.begin(),
                      parent_->data_.submap_data.at(submap_id).node_ids.end(),
                      nodes_to_retain.begin(), nodes_to_retain.end(),
                      std::inserter(nodes_to_remove, nodes_to_remove.begin()));

  // Remove all 'data_.constraints' related to 'submap_id'.
  {
    // Step: 1 删除submap_id相关的约束
    std::vector<Constraint> constraints;
    for (const Constraint& constraint : parent_->data_.constraints) {
      if (constraint.submap_id != submap_id) {
        constraints.push_back(constraint);
      }
    }
    parent_->data_.constraints = std::move(constraints);
  }

  // Remove all 'data_.constraints' related to 'nodes_to_remove'.
  // If the removal lets other submaps lose all their inter-submap constraints,
  // delete their corresponding constraint submap matchers to save memory.
  {
    std::vector<Constraint> constraints;
    std::set<SubmapId> other_submap_ids_losing_constraints;
    // Step: 2 删除与nodes_to_remove中节点相关联的约束, 并对submap_id进行标记
    for (const Constraint& constraint : parent_->data_.constraints) {
      if (nodes_to_remove.count(constraint.node_id) == 0) {
        constraints.push_back(constraint);
      } else {
        // A constraint to another submap will be removed, mark it as affected.
        other_submap_ids_losing_constraints.insert(constraint.submap_id);
      }
    }
    parent_->data_.constraints = std::move(constraints);
    // Go through the remaining constraints to ensure we only delete scan
    // matchers of other submaps that have no inter-submap constraints left.
    // 检查剩余的约束以确保我们只删除没有子图间约束的其他子图的扫描匹配器
    for (const Constraint& constraint : parent_->data_.constraints) {
      if (constraint.tag == Constraint::Tag::INTRA_SUBMAP) {
        continue;
      } 
      // 只要other_submap_ids_losing_constraints内submap_id还存在其他的子图间约束
      // 就把这个子图id从other_submap_ids_losing_constraints中删除, 可以留着
      else if (other_submap_ids_losing_constraints.count(
                     constraint.submap_id)) {
        // This submap still has inter-submap constraints - ignore it.
        other_submap_ids_losing_constraints.erase(constraint.submap_id);
      }
    }
    // Delete scan matchers of the submaps that lost all constraints.
    // TODO(wohe): An improvement to this implementation would be to add the
    // caching logic at the constraint builder which could keep around only
    // recently used scan matchers.
    // Step: 3 删除这些子图id的匹配器
    for (const SubmapId& submap_id : other_submap_ids_losing_constraints) {
      parent_->constraint_builder_.DeleteScanMatcher(submap_id);
    }
  }

  // Mark the submap with 'submap_id' as trimmed and remove its data.
  CHECK(parent_->data_.submap_data.at(submap_id).state ==
        SubmapState::kFinished);
  // Step: 4 删除这个子图的指针
  parent_->data_.submap_data.Trim(submap_id);
  // Step: 5 删除这个子图的匹配器, 与多分辨率地图
  parent_->constraint_builder_.DeleteScanMatcher(submap_id);
  // Step: 6 删除optimization_problem_中的这个子图
  parent_->optimization_problem_->TrimSubmap(submap_id);

  // We have one submap less, update the gauge metrics.
  kDeletedSubmapsMetric->Increment();
  if (parent_->IsTrajectoryFrozen(submap_id.trajectory_id)) {
    kFrozenSubmapsMetric->Decrement();
  } else {
    kActiveSubmapsMetric->Decrement();
  }

  // Remove the 'nodes_to_remove' from the pose graph and the optimization
  // problem.
  // Step: 7 删除节点
  for (const NodeId& node_id : nodes_to_remove) {
    parent_->data_.trajectory_nodes.Trim(node_id);
    parent_->optimization_problem_->TrimTrajectoryNode(node_id);
  }
}

/****************************************************************/

MapById<SubmapId, PoseGraphInterface::SubmapData>
PoseGraph2D::GetSubmapDataUnderLock() const {
  MapById<SubmapId, PoseGraphInterface::SubmapData> submaps;
  for (const auto& submap_id_data : data_.submap_data) {
    submaps.Insert(submap_id_data.id,
                   GetSubmapDataUnderLock(submap_id_data.id));
  }
  return submaps;
}

// 这里可以自己添加回调函数, 进行改进
void PoseGraph2D::SetGlobalSlamOptimizationCallback(
    PoseGraphInterface::GlobalSlamOptimizationCallback callback) {
  global_slam_optimization_callback_ = callback;
}

void PoseGraph2D::RegisterMetrics(metrics::FamilyFactory* family_factory) {
  auto* latency = family_factory->NewGaugeFamily(
      "mapping_2d_pose_graph_work_queue_delay",
      "Age of the oldest entry in the work queue in seconds");
  kWorkQueueDelayMetric = latency->Add({});
  auto* queue_size =
      family_factory->NewGaugeFamily("mapping_2d_pose_graph_work_queue_size",
                                     "Number of items in the work queue");
  kWorkQueueSizeMetric = queue_size->Add({});
  auto* constraints = family_factory->NewGaugeFamily(
      "mapping_2d_pose_graph_constraints",
      "Current number of constraints in the pose graph");
  kConstraintsDifferentTrajectoryMetric =
      constraints->Add({{"tag", "inter_submap"}, {"trajectory", "different"}});
  kConstraintsSameTrajectoryMetric =
      constraints->Add({{"tag", "inter_submap"}, {"trajectory", "same"}});
  auto* submaps = family_factory->NewGaugeFamily(
      "mapping_2d_pose_graph_submaps", "Number of submaps in the pose graph.");
  kActiveSubmapsMetric = submaps->Add({{"state", "active"}});
  kFrozenSubmapsMetric = submaps->Add({{"state", "frozen"}});
  kDeletedSubmapsMetric = submaps->Add({{"state", "deleted"}});
}

}  // namespace mapping
}  // namespace cartographer
