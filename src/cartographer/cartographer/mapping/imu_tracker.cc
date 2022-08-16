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

#include "cartographer/mapping/imu_tracker.h"

#include <cmath>
#include <limits>

#include "cartographer/common/math.h"
#include "cartographer/mapping/internal/eigen_quaterniond_from_two_vectors.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

/**
 * @brief Construct a new Imu Tracker:: Imu Tracker object
 * 
 * @param[in] imu_gravity_time_constant 这个值在2d与3d情况下都为10
 * @param[in] time 
 */
ImuTracker::ImuTracker(const double imu_gravity_time_constant,
                       const common::Time time)
    : imu_gravity_time_constant_(imu_gravity_time_constant),// 10s
      time_(time),// 当前时刻
      last_linear_acceleration_time_(common::Time::min()),// 上一次的线加速度为最小值
      orientation_(Eigen::Quaterniond::Identity()), // 初始方向角 [0, 0, 0, 1]，启动carto的一瞬间，机器人的姿态
      gravity_vector_(Eigen::Vector3d::UnitZ()),    // 重力方向初始化为[0,0,1]
      imu_angular_velocity_(Eigen::Vector3d::Zero())// imu角速度为0
      {}

/**
 * @brief 预测出time时刻的姿态与重力方向
 * 
 * @param[in] time 要预测的时刻
 */
void ImuTracker::Advance(const common::Time time) 
{

  CHECK_LE(time_, time);
  // 当前预测位姿的时间 - 上一次预测位姿的时间
  const double delta_t = common::ToSeconds(time - time_);

  // 上一时刻的角速度乘以时间,得到当前时刻相对于上一时刻的预测的姿态变化量,再转换成四元数Δq
  const Eigen::Quaterniond rotation =
      // 轴角转换成四元数
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(imu_angular_velocity_ * delta_t));// ω_last*Δtime = ΔΘ
          
  // R_current = R_last * ΔR: 使用上一时刻的姿态 orientation_ 乘以姿态变化量, 得到当前时刻的预测出的机器人姿态
  orientation_ = (orientation_ * rotation).normalized();// 更新机器人姿态

  // G_current = ΔR¯¹ * G_last: 机体旋转多少度，重力向量就要逆旋转多少度，保持始终竖直向上
  gravity_vector_ = rotation.conjugate() * gravity_vector_;
  
  // 更新时间 time_last = time_current
  time_ = time;
}

/**
 * @brief 更新线性加速度的值,并根据重力的方向对上一时刻的姿态进行校准
 * 
 * @param[in] imu_linear_acceleration imu的线加速度的大小
 */
void ImuTracker::AddImuLinearAccelerationObservation(
    const Eigen::Vector3d& imu_linear_acceleration) 
{
  // Update the 'gravity_vector_' with an exponential moving average using the
  // 'imu_gravity_time_constant'.
  // 指数滑动平均法 exponential moving average
 
  // step1 求Δt, Δt初始时刻为infinity, 之后为time_-last_linear_acceleration_time_
  const double delta_t =
      last_linear_acceleration_time_ > common::Time::min()
          ? common::ToSeconds(time_ - last_linear_acceleration_time_)
          : std::numeric_limits<double>::infinity();
  // 更新 time_last = time_current 
  last_linear_acceleration_time_ = time_;

  // step2 指数平均滤波，求α, α=1-e^(-Δt/10)，Δt越大, α越大
  const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);

  // step3 将之前的线加速度(gravity_vector_)与当前传入的IMU线加速度进行融合, 这里采用指数滑动平均法

  // 指数来确定权重, 因为有噪声的存在, 时间差越大, 当前的线性加速度(0,0,1)的权重越大
  // 时间间隔越大越相信imu_linear_acceleration，gravity_vector_是由IMU计算得出，短期有效
  gravity_vector_ =
      (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;



  // step4 求 线性加速度的值 与 由上一时刻姿态求出的线性加速度 间的旋转矫正量：rotation
  // 根据当前到线性加速度对之前的线性加速度进行矫正，矫正量：rotation 
  /*
  notice: 求解两个向量之间的旋转： 给定向量a、b，求解它们的旋转q，使得a = qb

  gravity_vector_ 与 orientation_，是相反的旋转，所以互逆
  更新之后的重力加速度与之前状态的重力加速度 之间的旋转量为rotation

  */
  const Eigen::Quaterniond rotation = FromTwoVectors(
      // a = 重力加速度, b = 机器人当前姿态¯¹*(0,0,1) = z轴往顺时针方向旋转
      gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());

  // step5 使用这个修正量来修正当前的机器人姿态，使得z轴与重力加速度方向重合
  orientation_ = (orientation_ * rotation).normalized();

  // notice: glog CHECK_GT: 第一个参数要大于第二个参数
  // 如果线性加速度与姿态均计算完全正确,那这二者的乘积应该是单位矩阵，互为逆
  CHECK_GT((orientation_ * gravity_vector_).z(), 0.);
  CHECK_GT((orientation_ * gravity_vector_).normalized().z(), 0.99);
}

// 更新角速度
void ImuTracker::AddImuAngularVelocityObservation(
    const Eigen::Vector3d& imu_angular_velocity) 
{
  imu_angular_velocity_ = imu_angular_velocity;
}

}  // namespace mapping
}  // namespace cartographer
