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

#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"

#include <limits>
#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/range_data.h"

namespace cartographer {
namespace mapping {

static auto* kLocalSlamLatencyMetric = metrics::Gauge::Null();
static auto* kLocalSlamRealTimeRatio = metrics::Gauge::Null();
static auto* kLocalSlamCpuRealTimeRatio = metrics::Gauge::Null();
static auto* kRealTimeCorrelativeScanMatcherScoreMetric =
    metrics::Histogram::Null();
static auto* kCeresScanMatcherCostMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualDistanceMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualAngleMetric = metrics::Histogram::Null();

/**
 * @brief 构造函数
 * 
 * @param[in] options 
 * @param[in] expected_range_sensor_ids 所有range类型传感器的话题
 */
LocalTrajectoryBuilder2D::LocalTrajectoryBuilder2D(
    const proto::LocalTrajectoryBuilderOptions2D& options,
    const std::vector<std::string>& expected_range_sensor_ids)
    : options_(options),
      // 使用trajectory_builder_2d.lua文件中的参数初始化
      active_submaps_(options.submaps_options()),
      motion_filter_(options_.motion_filter_options()),
      real_time_correlative_scan_matcher_(// 初始化相关性扫描匹配器
          options_.real_time_correlative_scan_matcher_options()),
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()),
      range_data_collator_(expected_range_sensor_ids) {}

LocalTrajectoryBuilder2D::~LocalTrajectoryBuilder2D() {}

/**
 * @brief 先进行点云的旋转与z方向的滤波, 然后再进行体素滤波减少数据量
 * 
 * @param[in] transform_to_gravity_aligned_frame R_track2grav * T_local2track
 * @param[in] range_data 传入的点云
 * @return sensor::RangeData 处理后的点云 拷贝
 */
sensor::RangeData
LocalTrajectoryBuilder2D::TransformToGravityAlignedFrameAndFilter(
    const transform::Rigid3f& transform_to_gravity_aligned_frame,
    const sensor::RangeData& range_data) const
{
  // 倾斜前机器人的姿态应该是单位矩阵，机器人倾斜后，激光散射出来的雷达数据，也是倾斜的，我们不要这样的数据，机器人倾斜多少，雷达数据反倾斜多少
  // 将点云旋转到与重力垂直的方向(通过机器人姿态R，把点云变成水平的), 再进行z轴上的过滤
  const sensor::RangeData cropped =
  // R_track2grav * T_local2track * P_local = R_track2grav * P_track = P_grav
  sensor::CropRangeData(sensor::TransformRangeData(
                            range_data, transform_to_gravity_aligned_frame),
                        options_.min_z(), options_.max_z()); // param: min_z max_z

  //  返回滤波后的点云，对点云进行体素滤波，如果点太密集，做一个降采样
  return sensor::RangeData{
      cropped.origin,
      //  param: voxel_filter_size = 0.025
      sensor::VoxelFilter(cropped.returns, options_.voxel_filter_size()),
      sensor::VoxelFilter(cropped.misses, options_.voxel_filter_size())};// TODO: P_track
}

/**
 * @brief 进行扫描匹配
 * 
 * @param[in] time 点云的时间
 * @param[in] pose_prediction IMU和Odom得到的先验位姿
 * @param[in] filtered_gravity_aligned_point_cloud 经过重力垂直，体素滤波后的点云(local坐标系下)，
 * @return std::unique_ptr<transform::Rigid2d> 匹配后的二维位姿
 */
std::unique_ptr<transform::Rigid2d> LocalTrajectoryBuilder2D::ScanMatch(
    const common::Time time, const transform::Rigid2d& pose_prediction,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud)// TODO: P_track
{
  if (active_submaps_.submaps().empty()) 
  {
    return absl::make_unique<transform::Rigid2d>(pose_prediction);
  }
  // 使用active_submaps_的第一个子图进行匹配
  std::shared_ptr<const Submap2D> matching_submap = active_submaps_.submaps().front();
  
  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.
  // 先验位姿
  transform::Rigid2d initial_ceres_pose = pose_prediction;

  // notice: 根据参数决定是否使用在线CSM(基于暴力搜索的CSM)对先验位姿进行校准
  // 如果使用：缺点计算复杂度高，但是在odom或者IMU不准时依然能达到很好的效果
  // real time CSM为ceres scan match提供一个很好的初值，之后用ceres scan match 进行优化
  if (options_.use_online_correlative_scan_matching())
  {
    // 基于暴力匹配的相关性扫描匹配器，校正后的后验位姿存入initial_ceres_pose，返回匹配得分
    const double score = real_time_correlative_scan_matcher_.Match(
        pose_prediction, filtered_gravity_aligned_point_cloud,// TODO: P_track
        *matching_submap->grid(), &initial_ceres_pose);// 先验位姿取地址，也是后验位姿的地址
    
    kRealTimeCorrelativeScanMatcherScoreMetric->Observe(score);
  }

  auto pose_observation = absl::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary summary;

  // notice: 暴力匹配校正后的后验位姿作为作为ceres优化的初值，使得匹配后得到的位姿更准确
  ceres_scan_matcher_.Match(pose_prediction.translation(),// 暴力匹配前的先验平移量
                            initial_ceres_pose,// 暴力匹配校正后后验位姿
                            filtered_gravity_aligned_point_cloud,// 体素滤波重力对齐后的一帧点云 TODO: P_track
                            *matching_submap->grid(),// 栅格地图
                            pose_observation.get(),// 扫描匹配后的后验位姿
                            &summary);
  
  // 一些度量
  if (pose_observation) 
  {
    kCeresScanMatcherCostMetric->Observe(summary.final_cost);
    const double residual_distance =
        (pose_observation->translation() - pose_prediction.translation())
            .norm();
    kScanMatcherResidualDistanceMetric->Observe(residual_distance);
    const double residual_angle =
        std::abs(pose_observation->rotation().angle() -
                 pose_prediction.rotation().angle());
    kScanMatcherResidualAngleMetric->Observe(residual_angle);
  }
  // 返回ceres计算后的后验位姿
  return pose_observation;
}

/**
 * @brief 处理点云数据, 进行扫描匹配, 将点云写成地图
 * 
 * @param[in] sensor_id 点云数据对应的话题名称
 * @param[in] unsynchronized_data 传入的未同步的测距数据
 * @return std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult> 匹配后的结果
 */
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddRangeData(
    const std::string& sensor_id,
    // 不同步的点云数据，unsynchronized_data是数据的别名、引用
    const sensor::TimedPointCloudData& unsynchronized_data)
{
  
  // step1 进行多个雷达点云数据的时间同步, 点云的坐标是相对于tracking_frame的
  auto synchronized_data =
      // 传入的是数据的引用
      range_data_collator_.AddRangeData(sensor_id, unsynchronized_data);

  
  if (synchronized_data.ranges.empty())
  {
    LOG(INFO) << "Range data collator filling buffer.";
    return nullptr;
  }

  const common::Time& time = synchronized_data.time;
  // 如果不用imu, 就在雷达这初始化位姿推测器
  if (!options_.use_imu_data()) 
  {
    InitializeExtrapolator(time);
  }

  if (extrapolator_ == nullptr) 
  {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return nullptr;
  }

  CHECK(!synchronized_data.ranges.empty());
  // TODO(gaschler): Check if this can strictly be 0.
  // 同步后最后一个点的相对时间戳为0
  CHECK_LE(synchronized_data.ranges.back().point_time.time, 0.f);

  // 计算第一个点的绝对时间 = 总时长 + 第一个点的相对时间
  const common::Time time_first_point =
      time +
      common::FromSeconds(synchronized_data.ranges.front().point_time.time);
  // 只有在extrapolator_初始化时, GetLastPoseTime()是common::Time::min()
  // 第一个点的绝对时间 < 上次校准的最后一个位姿的时间，位姿推测器无法往前推返回空
  if (time_first_point < extrapolator_->GetLastPoseTime())
  {
    LOG(INFO) << "Extrapolator is still initializing.";
    return nullptr;
  }
  // 激光雷达发射原点的位姿
  std::vector<transform::Rigid3f> range_data_poses;
  // 有多少个测距值就有多少个位姿
  range_data_poses.reserve(synchronized_data.ranges.size());
  bool warned = false;

  // 预测得到每一个时间点的位姿
  // 遍历每一个时间同步后的雷达数据
  for (const auto& range : synchronized_data.ranges) 
  {
    // 当前点的绝对时间
    common::Time time_point = time + common::FromSeconds(range.point_time.time);
    
    // 如果该时间比上次预测位姿的时间还要早,说明这个点的时间戳往回走了, 就报错
    // 当前点的绝对时间 < 上一次外推的时间
    if (time_point < extrapolator_->GetLastExtrapolatedTime()) 
    {
      // 一个循环只报一次错
      if (!warned) 
      {
        LOG(ERROR)
            << "Timestamp of individual range data point jumps backwards from "
            << extrapolator_->GetLastExtrapolatedTime() << " to " << time_point;
        warned = true;
      }
      // 上一次外推的时间
      time_point = extrapolator_->GetLastExtrapolatedTime();
    }
    
    // step2 预测出 每个点的时间戳时刻, tracking frame 在 local slam 坐标系下的坐标变换：T_tracking2local
    // 输入当前时间同步后的点云的绝对时间，返回time时刻下，预测得到的imu 在 local 坐标系下的位姿
    range_data_poses.push_back(
        extrapolator_->ExtrapolatePose(time_point).cast<float>());
  }

  // 支持多帧累积，但当前是一帧一帧的处理雷达数据
  if (num_accumulated_ == 0)
  {
    // 'accumulated_range_data_.origin' is uninitialized until the last
    // accumulation.
    // 多帧累积
    accumulated_range_data_ = sensor::RangeData{{}, {}, {}};
  }

  // Drop any returns below the minimum range and convert returns beyond the
  // maximum range into misses.
  // 遍历每一个时间同步后的测距值
  for (size_t i = 0; i < synchronized_data.ranges.size(); ++i) 
  {
    // 获取在tracking frame 下点的坐标，击中点
    const sensor::TimedRangefinderPoint& hit =
        synchronized_data.ranges[i].point_time;

    // 将点云的origins坐标(IMU坐标系下的雷达原点)转到 local slam 坐标系下
    const Eigen::Vector3f origin_in_local =
        range_data_poses[i] *
        synchronized_data.origins.at(synchronized_data.ranges[i].origin_index);
    
    // step3 点云运动畸变的去除, 同时将相对于tracking_frame的一帧点云坐标 转成 local坐标系下的坐标:
    sensor::RangefinderPoint hit_in_local =
        // p1_dist_track * T_track2pred * T_pred2local = p1_dist_track * T _track2local = p1_undist_local
        range_data_poses[i] * sensor::ToRangefinderPoint(hit);
    
    // 计算这个点到雷达的距离, 这里用的是去畸变之后的点的距离，点和雷达原点都在local坐标系下，(x,y) - (x',y')
    const Eigen::Vector3f delta = hit_in_local.position - origin_in_local;
    // 向量取模长 = 距离 √(Δx² + Δy²)
    const float range = delta.norm();
    
    // param: min_range max_range
    if (range >= options_.min_range()) // 0
    {
      if (range <= options_.max_range()) // 30
      {
        // 在这里可以看到, returns里保存的是local slam下的去畸变之后，并在测距范围之内的一帧点云坐标
        accumulated_range_data_.returns.push_back(hit_in_local);// 一个一个添加范围内的雷达数据到returns
      } 
      else//大于测距范围
      {
        // step4 超过max_range时的处理: 用一个距离进行替代, 并放入misses里 
        // param: missing_data_ray_length, 是个比例, 不是距离
        hit_in_local.position =
            origin_in_local + options_.missing_data_ray_length() / range * delta;
        accumulated_range_data_.misses.push_back(hit_in_local);// 一个一个添加范围之外的雷达数据到misses
      }
    }

  } // end for

  // 有一帧有效的数据了
  ++num_accumulated_;

  // param: num_accumulated_range_data = 1 几帧有效的点云数据进行一次扫描匹配
  if (num_accumulated_ >= options_.num_accumulated_range_data())
  {
    // 计算2次有效点云数据的的时间差
    const common::Time current_sensor_time = synchronized_data.time;
    absl::optional<common::Duration> sensor_duration;
    if (last_sensor_time_.has_value()) 
    {
      sensor_duration = current_sensor_time - last_sensor_time_.value();
    }
    last_sensor_time_ = current_sensor_time;

    // 重置变量
    num_accumulated_ = 0;

    // step5 R_track2gravity 重力系下，机器人的姿态，水平状态下为单位阵
    const transform::Rigid3d gravity_alignment = transform::Rigid3d::Rotation(
        extrapolator_->EstimateGravityOrientation(time));

    // 以一帧中最后一个点的时间戳估计出的坐标，作为这一帧点云数据的原点，t_track2local
    accumulated_range_data_.origin = range_data_poses.back().translation();
    
    // step6 扫描匹配 
    return AddAccumulatedRangeData(
        time,
        // 根据机器人姿态把点云旋转到与重力垂直的方向，并裁剪z方向上的点云，返回滤波后的点云：p_track or p_gravity
        TransformToGravityAlignedFrameAndFilter(
            // R_track2gravity * T_track2local¯¹ * P_local = R_track2gravity * T_local2track * P_local =  R_track2gravity * P_track = p_track or p_gravity
            gravity_alignment.cast<float>() * range_data_poses.back().inverse(),
            accumulated_range_data_),// local坐标系下的去畸变之后，并在测距范围之内的一帧点云坐标，P_local
        gravity_alignment,// R_track2gravity 重力系下，机器人的姿态，水平状态下为单位阵
        sensor_duration);
  }

  return nullptr;
}

/**
 * @brief 进行扫描匹配, 将点云写入地图
 * 
 * @param[in] time 点云的时间戳
 * @param[in] gravity_aligned_range_data 经过重力垂直、z方向裁剪、体素滤波后的点云(tracking坐标系下) p_track or p_gravity
 * @param[in] gravity_alignment R_track2gravity 重力系下，机器人的姿态，水平状态下为单位阵
 * @param[in] sensor_duration 2帧点云数据的时间差
 * @return std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult> 
 */
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddAccumulatedRangeData(
    const common::Time time,
    const sensor::RangeData& gravity_aligned_range_data,
    const transform::Rigid3d& gravity_alignment,
    const absl::optional<common::Duration>& sensor_duration)
{
  // 如果处理完点云之后数据为空, 就报错. 使用单线雷达时不要设置min_z
  if (gravity_aligned_range_data.returns.empty()) 
  {
    LOG(WARNING) << "Dropped empty horizontal range data.";
    return nullptr;
  }

  // Computes a gravity aligned pose prediction.计算重力对齐后的位姿
  // notice: 状态方程，进行位姿的预测, 没有经过重力对齐的位姿：里程计和imu估计出来的先验位姿：T_track2local
  const transform::Rigid3d non_gravity_aligned_pose_prediction =
      extrapolator_->ExtrapolatePose(time);

  // 计算重力对齐后的位姿，把机器人位姿假设为水平状态 
  // 未重力对齐的机器人位姿是倾斜的，将机器人变成水平状态(保证机器人在水平状态下做扫描匹配)，再转换为二维位姿
  // T_track2local * R_track2gravity¯¹ = T_track2local * R_gravity2track = T_gravity2local or T_track2local
  const transform::Rigid2d pose_prediction = transform::Project2D(
      non_gravity_aligned_pose_prediction * gravity_alignment.inverse());

  // step7 对 returns点云 进行自适应体素滤波，返回的点云的数据类型是PointCloud，P_track or p_gravity
  const sensor::PointCloud& filtered_gravity_aligned_point_cloud =
      sensor::AdaptiveVoxelFilter(gravity_aligned_range_data.returns,//  TODO: P_track or p_gravity
                                  options_.adaptive_voxel_filter_options());
                                  
  if (filtered_gravity_aligned_point_cloud.empty())
  {
    return nullptr;
  }
  /*
  notice: 真正的扫描匹配：
    local map frame <- gravity-aligned frame
    将自适应体素滤波后的点进行扫描匹配, 进行点云与submap的匹配
    第一帧栅格数据到来时，由于没有栅格地图，ScanMatch返回的是pose_prediction
    入参：预测的重力对齐后的位姿机器人先验位姿(处于水平状态)、重力对齐并体素滤波后的点云
    返回：后验位姿(水平状态下)
    先验概率经过似然概率修正之后，得到的后验概率其方差显著降低。
    一帧点云扫描匹配一个后验位姿，需要180帧点云写入一个子图，所以一个子图中有180个节点位姿
  */
  // notice: 观测方程 T_gravity2local or T_track2local 机器人被假设为水平状态下的位姿
  std::unique_ptr<transform::Rigid2d> pose_estimate_2d =
      ScanMatch(time, pose_prediction,// 位姿推测器推出的先验位姿：T_gravity2local or T_track2local 
      filtered_gravity_aligned_point_cloud);// TODO: P_track or p_gravity 水平状态下的点云

  if (pose_estimate_2d == nullptr)
  {
    LOG(WARNING) << "Scan matching failed.";
    return nullptr;
  }

  // 机器人原本是倾斜的，我们假想它为水平的，并对它的位姿进行优化后，需要还原为倾斜状态
  // T_gravity2local * R_track2gravity = T_track2local
  const transform::Rigid3d pose_estimate =
      transform::Embed3D(*pose_estimate_2d) * gravity_alignment;


  /*

  --------------------贝叶斯滤波思想----------------------
  预测步： 上一时刻的后验位姿 -- 状态方程(ExtrapolatePose) --> 预测出这一时刻的先验位姿
  更新步： 这一时刻的先验位姿 -- 观测方程(ScanMatch) --> 这一时刻的后验位姿
  (AddPose把这一时刻的后验位姿作为下一时刻的先验位姿传给状态方程去预测下下一时刻的位姿)

  */

  // notice: 使用ceres扫描匹配优化后的后验位姿，作为下一时刻的先验位姿传给外推器(状态方程去预测)
  extrapolator_->AddPose(time, pose_estimate);



  // 使用后验位姿来优化点云(都在水平状态下)，转换到local坐标系，为将点云写入子图做准备
  // T_track2local * P_track = P_local or T_gravity2local * P_gravity = P_local
  sensor::RangeData range_data_in_local =
      TransformRangeData(gravity_aligned_range_data,// P_track or P_gravity
                         transform::Embed3D(pose_estimate_2d->cast<float>()));// 水平状态下机器人的后验位姿
  

  // 将校正后的雷达数据写入submap，返回结构体：结构体中存放的一个智能指针和容器
  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
      time, range_data_in_local,// 后验位姿优化后的点云(local坐标系下，水平状态下)
      filtered_gravity_aligned_point_cloud,// 重力对齐、自适应体素滤波后的点云 P_track or p_gravity
      pose_estimate,// 倾斜状态的后验位姿
      gravity_alignment.rotation());// R_track2gravity

  // 计算耗时
  const auto wall_time = std::chrono::steady_clock::now();
  if (last_wall_time_.has_value()) 
  {
    const auto wall_time_duration = wall_time - last_wall_time_.value();
    kLocalSlamLatencyMetric->Set(common::ToSeconds(wall_time_duration));
    if (sensor_duration.has_value()) 
    {
      kLocalSlamRealTimeRatio->Set(common::ToSeconds(sensor_duration.value()) /
                                   common::ToSeconds(wall_time_duration));
    }
  }
  // 计算cpu耗时
  const double thread_cpu_time_seconds = common::GetThreadCpuTimeSeconds();
  if (last_thread_cpu_time_seconds_.has_value()) 
  {
    const double thread_cpu_duration_seconds =
        thread_cpu_time_seconds - last_thread_cpu_time_seconds_.value();
    if (sensor_duration.has_value()) {
      kLocalSlamCpuRealTimeRatio->Set(
          common::ToSeconds(sensor_duration.value()) /
          thread_cpu_duration_seconds);
    }
  }
  last_wall_time_ = wall_time;
  last_thread_cpu_time_seconds_ = thread_cpu_time_seconds;

  // 返回扫描匹配的结果：1.时间、2.经过扫描匹配之后位姿，校准之后的雷达数据、3.将激光插入栅格地图后的子图
  return absl::make_unique<MatchingResult>(
      MatchingResult{time, pose_estimate, std::move(range_data_in_local),
                     std::move(insertion_result)});// 初始化结构体
}

/**
 * @brief 将处理后的雷达数据写入submap
 * 
 * @param[in] time 点云的时间
 * @param[in] range_data_in_local 一帧体素滤波后的点云
 * @param[in] filtered_gravity_aligned_point_cloud 一帧自适应体素滤波后的点云
 * @param[in] pose_estimate 扫描匹配后的三维位姿
 * @param[in] gravity_alignment 机器人位姿
 * @return std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult> 
 */
std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult>
LocalTrajectoryBuilder2D::InsertIntoSubmap(
    const common::Time time, const sensor::RangeData& range_data_in_local,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment) 
{

  // notice: Motion filter still 如果移动距离过小, 或者时间过短, 丢掉当前帧点云，不进行子图的更新，
  if (motion_filter_.IsSimilar(time, pose_estimate)) 
  {
    // 返回空，不将点云插入到地图中
    return nullptr;
  }

  // notice: 如果通过了motion filter， 酒吧体素滤波后的点云插入到子图
  // 将一帧体素滤波后的点云数据写入到submap中，返回一个容器，容器中存放的是指向子图的智能指针
  std::vector<std::shared_ptr<const Submap2D>> insertion_submaps =
      active_submaps_.InsertRangeData(range_data_in_local);

  // 返回结构体：结构体中存放的一个智能指针和容器
  return absl::make_unique<InsertionResult>(InsertionResult{
      std::make_shared<const TrajectoryNode::Data>(TrajectoryNode::Data{
          time,
          gravity_alignment,
          filtered_gravity_aligned_point_cloud,  // 这里存的是一帧自适应体素滤波后的点云
          {},  // 'high_resolution_point_cloud' is only used in 3D.
          {},  // 'low_resolution_point_cloud' is only used in 3D.
          {},  // 'rotational_scan_matcher_histogram' is only used in 3D.
          pose_estimate}),
      std::move(insertion_submaps)});
}

// notice: 状态方程入口 将IMU数据加入到Extrapolator中
void LocalTrajectoryBuilder2D::AddImuData(const sensor::ImuData& imu_data)
{
  CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";

  InitializeExtrapolator(imu_data.time);

  extrapolator_->AddImuData(imu_data);
}

// notice: 状态方程入口 将里程计数据加入到Extrapolator中
void LocalTrajectoryBuilder2D::AddOdometryData(
    const sensor::OdometryData& odometry_data) 
{
  if (extrapolator_ == nullptr) 
  {
    // Until we've initialized the extrapolator we cannot add odometry data.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  extrapolator_->AddOdometryData(odometry_data);
}

// 如果Extrapolator没有初始化就进行初始化
void LocalTrajectoryBuilder2D::InitializeExtrapolator(const common::Time time) 
{
  // 如果已经初始化过了就直接返回
  if (extrapolator_ != nullptr) 
  {
    return;
  }

  // 注意 use_imu_based为true就会报错
  CHECK(!options_.pose_extrapolator_options().use_imu_based());
  // TODO(gaschler): Consider using InitializeWithImu as 3D does.

  // 初始化位姿推测器
  extrapolator_ = absl::make_unique<PoseExtrapolator>(
      ::cartographer::common::FromSeconds(
      options_.pose_extrapolator_options().constant_velocity().pose_queue_duration()), // 0.001s
      options_.pose_extrapolator_options().constant_velocity().imu_gravity_time_constant()); // 10s

  // 添加初始位姿，入参：当前IMU数据的时间和单位矩阵
  extrapolator_->AddPose(time, transform::Rigid3d::Identity());
}

void LocalTrajectoryBuilder2D::RegisterMetrics(
    metrics::FamilyFactory* family_factory) {
  auto* latency = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_latency",
      "Duration from first incoming point cloud in accumulation to local slam "
      "result");
  kLocalSlamLatencyMetric = latency->Add({});
  auto* real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_real_time_ratio",
      "sensor duration / wall clock duration.");
  kLocalSlamRealTimeRatio = real_time_ratio->Add({});

  auto* cpu_real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_cpu_real_time_ratio",
      "sensor duration / cpu duration.");
  kLocalSlamCpuRealTimeRatio = cpu_real_time_ratio->Add({});
  auto score_boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_scores", "Local scan matcher scores",
      score_boundaries);
  kRealTimeCorrelativeScanMatcherScoreMetric =
      scores->Add({{"scan_matcher", "real_time_correlative"}});
  auto cost_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 100);
  auto* costs = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_costs", "Local scan matcher costs",
      cost_boundaries);
  kCeresScanMatcherCostMetric = costs->Add({{"scan_matcher", "ceres"}});
  auto distance_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 10);
  auto* residuals = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_residuals",
      "Local scan matcher residuals", distance_boundaries);
  kScanMatcherResidualDistanceMetric =
      residuals->Add({{"component", "distance"}});
  kScanMatcherResidualAngleMetric = residuals->Add({{"component", "angle"}});
}

}  // namespace mapping
}  // namespace cartographer
