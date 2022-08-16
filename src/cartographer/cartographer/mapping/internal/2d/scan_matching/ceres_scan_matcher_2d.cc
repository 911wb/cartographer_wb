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

#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/internal/ceres_solver_options.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/rotation_delta_cost_functor_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/translation_delta_cost_functor_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/tsdf_match_cost_function_2d.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

proto::CeresScanMatcherOptions2D CreateCeresScanMatcherOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::CeresScanMatcherOptions2D options;
  options.set_occupied_space_weight(
      parameter_dictionary->GetDouble("occupied_space_weight"));
  options.set_translation_weight(
      parameter_dictionary->GetDouble("translation_weight"));
  options.set_rotation_weight(
      parameter_dictionary->GetDouble("rotation_weight"));
  *options.mutable_ceres_solver_options() =
      common::CreateCeresSolverOptionsProto(
          parameter_dictionary->GetDictionary("ceres_solver_options").get());
  return options;
}

// 初始化
CeresScanMatcher2D::CeresScanMatcher2D(
    const proto::CeresScanMatcherOptions2D& options)
    : options_(options),
      ceres_solver_options_(
          common::CreateCeresSolverOptions(options.ceres_solver_options())) 
{
  // 使用QR分解，求增量方程
  ceres_solver_options_.linear_solver_type = ceres::DENSE_QR;
}

CeresScanMatcher2D::~CeresScanMatcher2D() {}

/**
 * @brief 基于Ceres的扫描匹配
 * 
 * @param[in] target_translation 只有xy，暴力匹配前的先验平移量
 * @param[in] initial_pose_estimate 暴力匹配校正后的位姿, 有xy与theta
 * @param[in] point_cloud 用于匹配的点云 点云的原点位于local坐标系原点 体素滤波重力对齐后的一帧点云
 * @param[in] grid 用于匹配的栅格地图
 * @param[out] pose_estimate 储存优化之后的后验位姿
 * @param[out] summary 
 */
void CeresScanMatcher2D::Match(const Eigen::Vector2d& target_translation,
                               const transform::Rigid2d& initial_pose_estimate,
                               const sensor::PointCloud& point_cloud,// TODO: P_track
                               const Grid2D& grid,
                               transform::Rigid2d* const pose_estimate,
                               ceres::Solver::Summary* const summary) const 
{
  // 初始化待优化变量：暴力匹配校正 后的位姿
  double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                   initial_pose_estimate.translation().y(),
                                   initial_pose_estimate.rotation().angle()};
  ceres::Problem problem;

  // notice: 地图部分的残差
  CHECK_GT(options_.occupied_space_weight(), 0.);
  switch (grid.GetGridType()) 
  {
    
    case GridType::PROBABILITY_GRID:
      // 添加点云与栅格地图匹配的残差块
      problem.AddResidualBlock(
          CreateOccupiedSpaceCostFunction2D(// 返回代价函数
              options_.occupied_space_weight() /
                  std::sqrt(static_cast<double>(point_cloud.size())),// 地图的权重/这一帧点云的个数
              point_cloud,// TODO: P_track
              grid),// 用于匹配的栅格地图
          nullptr,// 核函数
          ceres_pose_estimate);// notice: 待优化变量的地址(暴力匹配校正后的位姿)，更新为点云与栅格地图匹配ceres优化后的位姿
      break;
    case GridType::TSDF:
      problem.AddResidualBlock(
          CreateTSDFMatchCostFunction2D(
              options_.occupied_space_weight() /
                  std::sqrt(static_cast<double>(point_cloud.size())),
              point_cloud, static_cast<const TSDF2D&>(grid)),
          nullptr /* 核函数 */, ceres_pose_estimate);
      break;
  }

  // -------------------------------平移和旋转残差可以注释掉，依然不影响------------------------------------
/*

  CHECK_GT(options_.translation_weight(), 0.);

  // notice: 添加平移的残差，作者认为位姿推测器推测出来的平移量是准的，ceres优化出来的是不准的 ??
  problem.AddResidualBlock(
      TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(// 返回代价函数
          options_.translation_weight(), // 平移权重
          target_translation), // 目标值：暴力匹配前的先验平移
      nullptr,// 核函数
      ceres_pose_estimate);// 待优化变量：点云与栅格地图匹配ceres优化后的平移(不是暴力匹配校正 后的位姿)

  CHECK_GT(options_.rotation_weight(), 0.);

  // notice: 添加旋转的残差, 固定了角度不变
  problem.AddResidualBlock(
      RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          options_.rotation_weight(),// 角度权重
          ceres_pose_estimate[2]),// 角度的目标值：点云与栅格地图匹配ceres优化后的角度
      nullptr,// 核函数
      ceres_pose_estimate);// 待优化变量：点云与栅格地图匹配ceres优化后的角度(不是暴力匹配校正 后的位姿)


*/


  // 根据配置进行求解
  ceres::Solve(ceres_solver_options_, &problem, summary);

  // 取出最终优化后的后验位姿
  *pose_estimate = transform::Rigid2d(
      {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
