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

#ifndef CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
#define CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_

#include <deque>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/imu_tracker.h"
#include "cartographer/mapping/pose_extrapolator_interface.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

// Keep poses for a certain duration to estimate linear and angular velocity.
// Uses the velocities to extrapolate motion. Uses IMU and/or odometry data if
// available to improve the extrapolation.
// 保持poses一定持续时间, 以估计线速度和角速度
// 使用速度预测运动. 使用IMU和/或里程计数据（如果有）来改善预测
class PoseExtrapolator : public PoseExtrapolatorInterface 
{
 public:
  //需要两个参数来初始化
  explicit PoseExtrapolator(common::Duration pose_queue_duration,
                            double imu_gravity_time_constant);

  PoseExtrapolator(const PoseExtrapolator&) = delete;
  PoseExtrapolator& operator=(const PoseExtrapolator&) = delete;

  static std::unique_ptr<PoseExtrapolator> InitializeWithImu(
      common::Duration pose_queue_duration, double imu_gravity_time_constant,
      const sensor::ImuData& imu_data);

  // c++11: override 关键字告诉编译器, 该函数应覆盖基类中的函数.
  // 如果该函数实际上没有覆盖任何函数, 则会导致编译器错误
  // 如果没加这个关键字 也没什么严重的error 只是少了编译器检查的安全性

  // Returns the time of the last added pose or Time::min() if no pose was added
  // yet.
  common::Time GetLastPoseTime() const override;
  common::Time GetLastExtrapolatedTime() const override;

  void AddPose(common::Time time, const transform::Rigid3d& pose) override;
  void AddImuData(const sensor::ImuData& imu_data) override;
  void AddOdometryData(const sensor::OdometryData& odometry_data) override;
  transform::Rigid3d ExtrapolatePose(common::Time time) override;

  ExtrapolationResult ExtrapolatePosesWithGravity(
      const std::vector<common::Time>& times) override;

  // Returns the current gravity alignment estimate as a rotation from
  // the tracking frame into a gravity aligned frame.
  Eigen::Quaterniond EstimateGravityOrientation(common::Time time) override;

 private:
  void UpdateVelocitiesFromPoses();
  void TrimImuData();
  void TrimOdometryData();
  void AdvanceImuTracker(common::Time time, ImuTracker* imu_tracker) const;
  Eigen::Quaterniond ExtrapolateRotation(common::Time time,
                                         ImuTracker* imu_tracker) const;
  Eigen::Vector3d ExtrapolateTranslation(common::Time time);

  // 保存一定时间内的pose
  const common::Duration pose_queue_duration_;
  struct TimedPose {
    common::Time time;
    transform::Rigid3d pose;
  };
  // 队列数据最少是2个
  std::deque<TimedPose> timed_pose_queue_;

  // ---------------------- 通过扫描匹配后的pose计算的速度 -----------------
  // 通过pose计算的线速度，只在添加位姿时更新，用于位姿预测时，不使用里程计数据时平移量预测 
  Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
  //  通过pose计算的角速度，不使用里程计数据时的角速度的更新
  Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();

  const double gravity_time_constant_;
  // 队列数据最少是1个
  std::deque<sensor::ImuData> imu_data_;

  // c++11: std::unique_ptr 是独享被管理对象指针所有权的智能指针
  // 它无法复制到其他 unique_ptr, 也无法通过值传递到函数,也无法用于需要副本的任何标准模板库 (STL) 算法
  // 只能通过 std::move() 来移动unique_ptr
  // std::make_unique 是 C++14 才有的特性
  
  // ----------------------- 姿态预测相关 -------------------------
  // 只在添加位姿addpose时更新，用于保存添加校准位姿时的姿态，指向ImuTracker的智能指针
  // 该对象的属性有，姿态、重力向量、imu角速度、线加速度
  std::unique_ptr<ImuTracker> imu_tracker_;
  // 只在添加位姿addpose时更新，用于根据里程计数据计算线速度时的位姿预测
  std::unique_ptr<ImuTracker> odometry_imu_tracker_; 
  // 只在添加位姿addpose时更新，用于预测姿态的ImuTracker
  std::unique_ptr<ImuTracker> extrapolation_imu_tracker_; 
  
  TimedPose cached_extrapolated_pose_;

  // 队列数据最少是2个
  std::deque<sensor::OdometryData> odometry_data_;

  // -----------------通过里程计计算的线速度和角速度----------
  // 只在添加里程计数据时更新，用于姿态预测时的平移量预测
  Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();
  // 只在添加里程计数据时更新，用于不使用IMU数据时的角速度的更新
  Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();

  /*
  预测位姿时的四种情况都是匀速运用模型

  使用的传感器              平移预测                                             姿态预测
  IMU、里程计          通过数据队列的开始和末尾两个数据计算出的v乘以Δtime         通过imu的角速度ω乘以Δtime
  imu       通过扫描匹配后的pose数据队列的开始和末尾两个数据计算出的v乘以Δtime     通过imu的角速度ω乘以Δtime
  里程计              通过数据队列的开始和末尾两个数据计算出的v乘以Δtime          通过里程计数据队列开始和末尾的两个数据计算出的角速度ω乘以Δtime
  NULL      通过扫描匹配后的pose数据队列的开始和末尾两个数据计算出的v乘以Δtime     通过扫描匹配后的pose数据队列的开始和末尾两个数据计算出的角速度ω乘以Δtime

  总结
    预测平移时: 有里程计就用里程计的线速度, 没有里程计就用pose计算的线速度进行预测
    预测姿态时: 有IMU就用IMU的角速度, 没有IMU时, 如果有里程计就用里程计计算出的角速
    度, 没有里程计就用pose计算的角速度进行预测，预测的都是相对值(增量), 要加上最后一个pose的位姿

  */
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
