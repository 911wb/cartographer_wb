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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_2D_H_

#include "Eigen/Core"
#include "ceres/ceres.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// Computes the cost of translating 'pose' to 'target_translation'.
// Cost increases with the solution's distance from 'target_translation'.
class TranslationDeltaCostFunctor2D 
{
 public:
  // 静态成员函数(使用类作用域来调用), 返回CostFunction
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor, const Eigen::Vector2d& target_translation) 
  {
    // 自动求导：<CostFuncor,残差的维度,参数的维度>
    return new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor2D,
                                           2 /* residuals */,
                                           3 /* pose variables */>(
        // 构造函数初始化
        new TranslationDeltaCostFunctor2D(scaling_factor, target_translation));
  }

  // 只进行平移的约束，平移量残差的计算, (pose[0] - x_)的平方ceres会自动加上，仿函数
  template <typename T>// pose是待优化的变量，即ceres优化后的位姿
  bool operator()(const T* const pose, T* residual) const 
  {

    // notice: 暴力匹配后的位姿 -> 暴力匹配前的位姿(目标) 
    // 残差1，f1 = 10*(x_ceres - x_target_init)
    residual[0] = scaling_factor_ * (pose[0] - x_);
    // 残差2，f2 = 10*(y_ceres - y_target)
    residual[1] = scaling_factor_ * (pose[1] - y_);
    return true;
  }

 private:
  // Constructs a new TranslationDeltaCostFunctor2D from the given
  // 'target_translation' (x, y).

  // 构造函数初始化
  explicit TranslationDeltaCostFunctor2D(
      const double scaling_factor, const Eigen::Vector2d& target_translation)
      : scaling_factor_(scaling_factor),
        x_(target_translation.x()),
        y_(target_translation.y()) {}

  TranslationDeltaCostFunctor2D(const TranslationDeltaCostFunctor2D&) = delete;
  TranslationDeltaCostFunctor2D& operator=(
      const TranslationDeltaCostFunctor2D&) = delete;

  const double scaling_factor_;
  // 使用暴力匹配前的先验平移量来初始化
  const double x_;
  const double y_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_2D_H_
