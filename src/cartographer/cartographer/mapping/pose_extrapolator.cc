/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/mapping/pose_extrapolator.h"

#include <algorithm>

#include "absl/memory/memory.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

/**
 * @brief 构造函数
 * 
 * @param[in] pose_queue_duration 时间差 0.001s
 * @param[in] imu_gravity_time_constant 10
 */
PoseExtrapolator::PoseExtrapolator(const common::Duration pose_queue_duration,
                                   double imu_gravity_time_constant)
    : pose_queue_duration_(pose_queue_duration),// 时间差 0.001s
      gravity_time_constant_(imu_gravity_time_constant),// 10S
      cached_extrapolated_pose_{common::Time::min(),
                                transform::Rigid3d::Identity()} {}

// 使用imu数据进行PoseExtrapolator的初始化，未用
std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithImu(
    const common::Duration pose_queue_duration,
    const double imu_gravity_time_constant, const sensor::ImuData& imu_data) 
{
  auto extrapolator = absl::make_unique<PoseExtrapolator>(
      pose_queue_duration, imu_gravity_time_constant);
  
  extrapolator->AddImuData(imu_data);
  extrapolator->imu_tracker_ =
      absl::make_unique<ImuTracker>(imu_gravity_time_constant, imu_data.time);
  extrapolator->imu_tracker_->AddImuLinearAccelerationObservation(
      imu_data.linear_acceleration);
  extrapolator->imu_tracker_->AddImuAngularVelocityObservation(
      imu_data.angular_velocity);
  extrapolator->imu_tracker_->Advance(imu_data.time);
  extrapolator->AddPose(
      imu_data.time,
      transform::Rigid3d::Rotation(extrapolator->imu_tracker_->orientation()));
  return extrapolator;
}

// 返回上次校准位姿的时间
common::Time PoseExtrapolator::GetLastPoseTime() const 
{
  // 如果尚未添加任何位姿, 则返回Time::min()
  if (timed_pose_queue_.empty())
  {
    return common::Time::min();
  }
  // 返回扫描匹配的最后一个位姿的时间
  return timed_pose_queue_.back().time;
}

// 获取上一次预测位姿的时间
common::Time PoseExtrapolator::GetLastExtrapolatedTime() const 
{
  if (!extrapolation_imu_tracker_) 
  {
    return common::Time::min();
  }
  return extrapolation_imu_tracker_->time();
}
/*
  将扫描匹配后的pose加入到pose队列中,计算线速度与角速度,并将imu_tracker_的状态更新到time时刻
  添加位姿时，没有用扫描匹配后的pose，对imutracker_进行校准，也没有对整体位姿进行校准
*/
void PoseExtrapolator::AddPose(const common::Time time,
                               const transform::Rigid3d& pose) 
{
  // 如果imu_tracker_没有初始化就先进行初始化
  if (imu_tracker_ == nullptr)
  {
    // 当前imu数据的时间
    common::Time tracker_start = time;
    if (!imu_data_.empty()) 
    {
      // 开始时间为最小时间
      tracker_start = std::min(tracker_start, imu_data_.front().time);
    }

    // imu_tracker_的初始化
    imu_tracker_ = absl::make_unique<ImuTracker>(gravity_time_constant_, tracker_start);
  }

  // 在timed_pose_queue_中保存scanMatch的位姿，越往后的数据越是新添加的数据
  timed_pose_queue_.push_back(TimedPose{time, pose});

  // 删除过时数据
  // 保持pose队列中第二个pose的时间要大于 当前的时间time - pose_queue_duration_
  while (timed_pose_queue_.size() > 2 && // timed_pose_queue_最少是2个数据
         timed_pose_queue_[1].time <= time - pose_queue_duration_)
  {
    timed_pose_queue_.pop_front();
  }

  // 根据加入的pose计算线速度与角速度
  UpdateVelocitiesFromPoses();

  // 使用IMU数据，将上一帧的姿态(上一时刻)，更新到当前时刻的姿态
  AdvanceImuTracker(time, imu_tracker_.get());

  // pose队列更新了,之前imu及里程计数据已经过时了
  // 因为pose是匹配的结果,之前的imu及里程计数据是用于预测的,现在结果都有了,之前的用于预测的数据肯定不需要了
  TrimImuData();// 双端队列可以从队列的头部删除时间最早的数据

  TrimOdometryData();

  // 将更新到当前时刻姿态的imu_tracker_，调用拷贝构造函数赋给odometry

  // 用于根据里程计数据计算线速度时姿态的预测
  odometry_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);
  
  // 用于位姿预测时的姿态预测
  extrapolation_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);
}

// 向imu数据队列中添加imu数据,并进行数据队列的修剪
void PoseExtrapolator::AddImuData(const sensor::ImuData& imu_data) 
{
  CHECK(timed_pose_queue_.empty() ||
        imu_data.time >= timed_pose_queue_.back().time);
  
  imu_data_.push_back(imu_data);

  // 双端队列可以从队列的头部删除时间最早的数据
  TrimImuData();
}

// 向odom数据队列中添加odom数据,并进行数据队列的修剪,并计算角速度与线速度
void PoseExtrapolator::AddOdometryData(
    const sensor::OdometryData& odometry_data)
{

  CHECK(timed_pose_queue_.empty() ||
        odometry_data.time >= timed_pose_queue_.back().time);
  odometry_data_.push_back(odometry_data);

  // 修剪odom的数据队列
  TrimOdometryData();
  // 数据队列中至少有2个数据
  if (odometry_data_.size() < 2) 
  {
    return;
  }

  // TODO(whess): Improve by using more than just the last two odometry poses.
  // Compute extrapolation in the tracking frame.
  // 取最新与最老的两个里程计数据
  const sensor::OdometryData& odometry_data_oldest = odometry_data_.front();
  const sensor::OdometryData& odometry_data_newest = odometry_data_.back();
  // 最新与最老odom数据间的时间差
  const double odometry_time_delta =
      common::ToSeconds(odometry_data_oldest.time - odometry_data_newest.time);

  // 两个位姿间的坐标变换
  // ΔT = T_newest¯¹ * T_oldest ：角度变化量除以时间得到角速度得到 tracking frame 在 local坐标系下的角速度
  const transform::Rigid3d odometry_pose_delta =
      odometry_data_newest.pose.inverse() * odometry_data_oldest.pose;

  // ΔT --> ΔR --> ΔΘ， 角速度ω = ΔΘ/Δt，两个位姿间的旋转量除以时间得到 tracking frame 的角速度
  angular_velocity_from_odometry_ =
      transform::RotationQuaternionToAngleAxisVector(
          odometry_pose_delta.rotation()) / odometry_time_delta;
  
  if (timed_pose_queue_.empty()) 
  {
    return;
  }

  // ΔT --> Δt，v_tracking_odom = Δt/Δtime: 平移量除以时间得到 相对于tracking frame的线速度, 
  // 2D机器人情况下只在x方向有数值，如果是3D情况下，x、y、z方向都有
  const Eigen::Vector3d
      linear_velocity_in_tracking_frame_at_newest_odometry_time =
          odometry_pose_delta.translation() / odometry_time_delta;


  // R_newest = R_last * ΔR: 根据位姿队列中最后一个位姿 乘以 上次添加位姿时的姿态预测到time时刻的姿态变化量 
  // 得到预测的最新里程计数据时刻 tracking frame 在 local 坐标系下的姿态
  // 后验的姿态(上一时刻) * odom预测姿态ΔR = 当前时刻的姿态 
  const Eigen::Quaterniond orientation_at_newest_odometry_time =
      timed_pose_queue_.back().pose.rotation() *
      ExtrapolateRotation(odometry_data_newest.time,
                          odometry_imu_tracker_.get());

  // 将tracking frame的线速度进行旋转, 得到 local 坐标系下 tracking frame 的线速度 
  // R_tracking2local * v_tracking_odom = v_local
  linear_velocity_from_odometry_ =
      orientation_at_newest_odometry_time *
      linear_velocity_in_tracking_frame_at_newest_odometry_time;
}

// 预测得到time时刻，imu 在 local 坐标系下的位姿
// 入参：当前时间同步后的点云的绝对时间
transform::Rigid3d PoseExtrapolator::ExtrapolatePose(const common::Time time) 
{
  // notice: 前端最新的位姿结果(后验位姿)
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();

  CHECK_GE(time, newest_timed_pose.time);

  // 如果本次预测时间与上次计算时间相同 就不再重复计算
  if (cached_extrapolated_pose_.time != time)
  {

    // 预测tracking frame在local坐标系下当前time时刻的位置
    const Eigen::Vector3d translation =
        // Δt_local + t_local  = 里程计平移预测量 上一时刻最新pose的坐标位置  = 机器人当前在time时刻的位置预测值(local坐标系下)
        ExtrapolateTranslation(time) + newest_timed_pose.pose.translation();
  
    // 预测机器人或imu在local坐标系下time时刻的姿态
    const Eigen::Quaterniond rotation =
        // R_tracking2local * ΔR_tracin2local = imu当前在time时刻的姿态预测值(local坐标系下)， ΔR = R_last¯¹ *  R_crru
        newest_timed_pose.pose.rotation() *
        ExtrapolateRotation(time, extrapolation_imu_tracker_.get());
    
    cached_extrapolated_pose_ =
        TimedPose{time, transform::Rigid3d{translation, rotation}};
  }

  return cached_extrapolated_pose_.pose;
}

Eigen::Quaterniond PoseExtrapolator::EstimateGravityOrientation(
    const common::Time time) 
{
  ImuTracker imu_tracker = *imu_tracker_;
  // 使得 imu_tracker 预测到time时刻
  AdvanceImuTracker(time, &imu_tracker);
  // 只返回R
  return imu_tracker.orientation();
}

// 根据pose队列计算tracking frame 在 local坐标系下的线速度与角速度
void PoseExtrapolator::UpdateVelocitiesFromPoses() 
{
  if (timed_pose_queue_.size() < 2)
  {
    // We need two poses to estimate velocities.
    return;
  }

  CHECK(!timed_pose_queue_.empty());

  // 取出队列最末尾的一个 Pose,也就是最新时间点的 Pose,并记录相应的时间
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const auto newest_time = newest_timed_pose.time;

  // 取出队列最开头的一个 Pose, 也就是最旧时间点的 Pose,并记录相应的时间
  const TimedPose& oldest_timed_pose = timed_pose_queue_.front();
  const auto oldest_time = oldest_timed_pose.time;

  // 计算两者的时间差:现在最新的时间- 以前最旧的时间，Δtime
  const double queue_delta = common::ToSeconds(newest_time - oldest_time);
  
  // 如果时间差小于pose_queue_duration_(1ms), 不进行计算
  if (queue_delta < common::ToSeconds(pose_queue_duration_)) 
  {
    LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: "
                 << queue_delta << " s";
    return;
  }

  // 获取scanMatch后的位姿，在local坐标系下
  const transform::Rigid3d& newest_pose = newest_timed_pose.pose;
  const transform::Rigid3d& oldest_pose = oldest_timed_pose.pose;

  // v = t_newest - t_oldest/ Δtime：平移量除以时间差得到 tracking frame 在 local坐标系下的线速度
  linear_velocity_from_poses_ = 
      (newest_pose.translation() - oldest_pose.translation()) / queue_delta;

  // ΔR = R_oldest¯¹ * R_newest --> ΔΘ，ω = ΔΘ/Δt：角度变化量除以时间得到角速度得到 tracking frame 在 local坐标系下的角速度
  angular_velocity_from_poses_ =
      transform::RotationQuaternionToAngleAxisVector(
          oldest_pose.rotation().inverse() * newest_pose.rotation()) / queue_delta;
}

// 修剪imu的数据队列,从imu_data_中丢掉过时的imu数据
void PoseExtrapolator::TrimImuData()
{
  // 保持imu队列中第二个数据的时间要大于最后一个后验位姿的时间, imu_date_最少是1个
  while (imu_data_.size() > 1 && !timed_pose_queue_.empty() &&
         imu_data_[1].time <= timed_pose_queue_.back().time)// 说明数据已经使用完毕
  {
    // 双端队列可以从队列的头部删除时间最早的数据
    // 假设只有两个数据，那么第二数据的时间如果小于后验位姿的时间，说明第一个数据也小于，第一个数据已经用过了
    imu_data_.pop_front();
  }
}

// 修剪odom的数据队列，从odometry_data_中丢掉过时的odom数据
void PoseExtrapolator::TrimOdometryData() 
{
  // 保持odom队列中第二个数据的时间要大于最后一个位姿的时间, odometry_data_最少是2个
  while (odometry_data_.size() > 2 && !timed_pose_queue_.empty() &&
         odometry_data_[1].time <= timed_pose_queue_.back().time) 
  {
    odometry_data_.pop_front();
  }
}

/**
 * @brief 更新imu_tracker的状态, 并将imu_tracker的状态预测到time时刻
 * 
 * @param[in] time 要预测到的时刻
 * @param[in] imu_tracker 给定的先验状态 
 */
// ExtrapolateRotation入参：extrapolation_imu_tracker_.get()
// AddPose、EstimateGravityOrientation下，入参:imu_tracker_.get()
void PoseExtrapolator::AdvanceImuTracker(const common::Time time,// 当前时刻
                                         ImuTracker* const imu_tracker) const 
{
  // 检查指定时间是否大于等于 ImuTracker 的时间
  CHECK_GE(time, imu_tracker->time());

  // 不使用imu 或者 预测时间之前没有imu数据的情况，数据没来就预测，后面才来
  if (imu_data_.empty() || time < imu_data_.front().time)
  {

    // 在time之前没有IMU数据, 因此我们推进ImuTracker, 并使用姿势和假重力产生的角速度来帮助2D稳定
    
    // 预测当前时刻的姿态与重力方向
    imu_tracker->Advance(time);

    // 使用理想的重力数据(0,0,1)┴ 对重力加速度进行更新，对机器人姿态进行修正
    // 没有imu数据，在2D平面运行，只有横摆角，使用(0,0,1)┴，代替重力加速度
    imu_tracker->AddImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ());
    

    // 没有imu数据 只能依靠其他方式得到的角速度进行测量值的更新
    imu_tracker->AddImuAngularVelocityObservation(
        odometry_data_.size() < 2 ? angular_velocity_from_poses_
                                  : angular_velocity_from_odometry_);
    return;
  }

  // 如果当前imu_tracker的时间比imu数据队列中第一个数据的时间早
  if (imu_tracker->time() < imu_data_.front().time) 
  {
    // 预测到imu数据队列中第一个数据对应当前时刻的姿态与重力方向
    imu_tracker->Advance(imu_data_.front().time);
  }

  // c++11: std::lower_bound() 是在区间内找到第一个大于等于 value = imu_tracker->time() 的值的位置并返回, 如果没找到就返回 end() 位置
  // 在第四个参数位置可以自定义比较规则,在区域内查找第一个 **不符合** comp 规则的元素

  // 在imu数据队列中找到第一个不满足lambda表达式的数据索引，大于等于 imu_tracker->time() 的数据的索引
  auto it = std::lower_bound(
      imu_data_.begin(), imu_data_.end(), imu_tracker->time(),
      [](const sensor::ImuData& imu_data, const common::Time& time)
      {
        return imu_data.time < time;// 小于当前时刻
      });

  // notice: 然后依次对imu数据进行预测, 以及添加观测, 直到用完这组数据或imu_data_的时间大于等于当前时间截止
  while (it != imu_data_.end() && it->time < time)// 有数据的时间 < 当前时间
  {
    // 预测出imu当前时刻的姿态与重力方向
    imu_tracker->Advance(it->time);

    // 根据imu线速度的观测,更新重力加速度的方向,并根据重力加速度的方向对上一时刻预测的姿态进行修正
    imu_tracker->AddImuLinearAccelerationObservation(it->linear_acceleration);

    // 利用imu数据更新角速度观测
    imu_tracker->AddImuAngularVelocityObservation(it->angular_velocity);
    ++it;
  }

  // 最后将imu_tracker的状态预测到当前time时刻
  imu_tracker->Advance(time);
}

// 计算从imu_tracker到time时刻的姿态变化量
// 入参：extrapolation_imu_tracker_.get()
Eigen::Quaterniond PoseExtrapolator::ExtrapolateRotation(
    const common::Time time, ImuTracker* const imu_tracker) const 
{
  // time >= 
  CHECK_GE(time, imu_tracker->time());

  // 更新imu_tracker(extrapolation_imu_tracker_)的状态到当前time时刻
  // 对上一帧的姿态(上一时刻的姿态)，更新到当前时刻的姿态
  AdvanceImuTracker(time, imu_tracker);

  // 通过imu_tracker_获取上一次位姿校准时的姿态
  const Eigen::Quaterniond last_orientation = imu_tracker_->orientation();
  
  // 求取上一帧到当前时刻预测出的姿态变化量：上一时刻机器人位姿的逆 * 当前时刻重力对齐后机器人位姿
  // ΔR_tracking2local = R_last¯¹ *  R_crru_tracking2local
  return last_orientation.inverse() * imu_tracker->orientation();
}

// 返回从最后一个位姿的时间 到当前time时刻 的imu在local坐标系下的平移量
Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(common::Time time) 
{
  // 前端最新位姿结果(后验位姿)
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();


  // 当前点的绝对时间 - 最新pose的时间 = Δt
  const double extrapolation_delta =
      common::ToSeconds(time - newest_timed_pose.time);

  // 使用tracking frame 在 local坐标系下的线速度 乘以时间 得到平移量的预测
  // 里程计的数据不够多
  if (odometry_data_.size() < 2)
  {
    // 平移预测量 = Δt*v_pose
    return extrapolation_delta * linear_velocity_from_poses_;
  }

  // 如果里程计数据足够多，就使用里程计计算出的速度，Δt_local = Δtime*v_odom_local
  return extrapolation_delta * linear_velocity_from_odometry_;
}

// 获取一段时间内的预测位姿的结果
PoseExtrapolator::ExtrapolationResult
PoseExtrapolator::ExtrapolatePosesWithGravity(
    const std::vector<common::Time>& times) 
{
  std::vector<transform::Rigid3f> poses;

  // c++11: std::prev 获取一个距离指定迭代器 n 个元素的迭代器,而不改变输入迭代器的值
  // 默认 n 为1,当 n 为正数时, 其返回的迭代器将位于 it 左侧；
  // 反之, 当 n 为负数时, 其返回的迭代器位于 it 右侧

  // 获取 [0, n-1] 范围的预测位姿
  for (auto it = times.begin(); it != std::prev(times.end()); ++it) 
  {
    poses.push_back(ExtrapolatePose(*it).cast<float>());
  }

  // 进行当前线速度的预测
  const Eigen::Vector3d current_velocity = odometry_data_.size() < 2
                                               ? linear_velocity_from_poses_
                                               : linear_velocity_from_odometry_;
  return ExtrapolationResult{poses,// 之前的n-1个位姿 previous_poses;
                             ExtrapolatePose(times.back()),// 第n个位姿，当前时刻的位姿
                             current_velocity,// 当前线速度
                             EstimateGravityOrientation(times.back())};

}

}  // namespace mapping
}  // namespace cartographer
