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

#include "cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.h"

#include "cartographer/mapping/probability_values.h"
#include "ceres/cubic_interpolation.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

// Computes a cost for matching the 'point_cloud' to the 'grid' with
// a 'pose'. The cost increases with poorer correspondence of the grid and the
// point observation (e.g. points falling into less occupied space).
class OccupiedSpaceCostFunction2D {
 public:
  // 构造函数初始化
  OccupiedSpaceCostFunction2D(const double scaling_factor,
                              const sensor::PointCloud& point_cloud,
                              const Grid2D& grid)
      : scaling_factor_(scaling_factor),
        point_cloud_(point_cloud),// TODO: P_track
        grid_(grid) {}

  template <typename T>

  // pose是待优化的变量，暴力匹配优化后的位姿
  bool operator()(const T* const pose, T* residual) const 
  {
    // 暴力匹配后的平移
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    // 暴力匹配后的旋转
    Eigen::Rotation2D<T> rotation(pose[2]);
    // 2*2的旋转矩阵
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform;
    /*
    暴力匹配优化后的T(二维的):
      R1 R2 tx
      R3 R4 ty
      0  0  1
    */
    // 待优化的位姿
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);
    
    // 网格适配器
    const GridArrayAdapter adapter(grid_);
    // 双立方插值器，在网格上插值
    ceres::BiCubicInterpolator<GridArrayAdapter> interpolator(adapter);

    const MapLimits& limits = grid_.limits();

    // 遍历每一个点
    for (size_t i = 0; i < point_cloud_.size(); ++i) 
    {
      // 2D点云坐标，(x,y,1)
      const Eigen::Matrix<T, 3, 1> point((T(point_cloud_[i].position.x())),
                                         (T(point_cloud_[i].position.y())),
                                         T(1.));// 权重

      // 根据预测位姿对单个点进行坐标变换，T_track2local * P_track = P_local ?? local的原点是submap的原点
      // 待优化变量为transform
      const Eigen::Matrix<T, 3, 1> world = transform * point;

      
      // 物理坐标转换为像素坐标，获取三次插值之后的栅格free值, Evaluate函数内部调用了GetValue函数
      interpolator.Evaluate(
          (limits.max().x() - world[0]) / limits.resolution() - 0.5 +
              static_cast<double>(kPadding),// x方向的像素坐标
          (limits.max().y() - world[1]) / limits.resolution() - 0.5 +
              static_cast<double>(kPadding),// y方向的像素坐标
          &residual[i]);// 像素坐标对应的空闲值：e = 1 - M(T*p)
      // free值越小, 表示占用的概率越大
      residual[i] = scaling_factor_ * residual[i];
    }
    /*

    输入一个点云的2D坐标P：
    1.对这个点的坐标进行双线性插值得到这个点的占据栅格值M(Pm)
      栅格值代表这个点的得分，最大为1，越大越好
      当所有的点都落在栅格上，那么得分最高
    2.求地图对这个点的梯度：
      ∂M/∂x，∂M/∂y
    3.把雷达坐标系下的2D点云转换到世界(地图)坐标系下
        T_track2world * p
    4.求该点的空闲栅格值作为残差
      e = 1 - M(T*p)
    5.开始优化
      要优化(求偏导)的变量：二维的T，即 T = (px, py, Yaw)
      根据增量方程求出ΔT
      求出增量后，令T _k+1 =  T _k + Tξ_k，不断迭代，使得目标函数朝着极小值方向下降，即不断调整位姿，使得点云落在占据栅格上
      停止迭代的条件：ΔT->0，想让更多点云落在占据栅格上的位姿增量，该增量越小残差也就越小

    */

    return true;
  }

 private:
  static constexpr int kPadding = INT_MAX / 4;
  
  // 自定义网格
  class GridArrayAdapter 
  {
   public:
    // 枚举 DATA_DIMENSION 表示被插值的向量或者函数的维度
    enum { DATA_DIMENSION = 1 };
    // 构造
    explicit GridArrayAdapter(const Grid2D& grid) : grid_(grid) {}

    // 获取栅格free值
    void GetValue(const int row, const int column, double* const value) const 
    {
      // 处于地图外部时, 赋予最大free值
      if (row < kPadding || column < kPadding || row >= NumRows() - kPadding ||
          column >= NumCols() - kPadding)
      {
        *value = kMaxCorrespondenceCost;
      }
      // 根据索引获取free值
      else 
      {
        *value = static_cast<double>(grid_.GetCorrespondenceCost(
            Eigen::Array2i(column - kPadding, row - kPadding)));
      }
    }


    // map上下左右各增加 kPadding
    int NumRows() const 
    {
      return grid_.limits().cell_limits().num_y_cells + 2 * kPadding;
    }

    int NumCols() const {
      return grid_.limits().cell_limits().num_x_cells + 2 * kPadding;
    }

   private:
    const Grid2D& grid_;
  };

  OccupiedSpaceCostFunction2D(const OccupiedSpaceCostFunction2D&) = delete;
  OccupiedSpaceCostFunction2D& operator=(const OccupiedSpaceCostFunction2D&) =
      delete;

  const double scaling_factor_;
  const sensor::PointCloud& point_cloud_;
  const Grid2D& grid_;
};

}  // namespace

// 工厂函数, 返回地图的CostFunction
ceres::CostFunction* CreateOccupiedSpaceCostFunction2D(
    const double scaling_factor, const sensor::PointCloud& point_cloud,// TODO: P_track
    const Grid2D& grid)
{
  // 自动求导 <costfuncto,Dynamic,3> 点云的维度是变换的所以残差的维度也是变换的，待优化变量(位姿)的维度
  return new ceres::AutoDiffCostFunction<OccupiedSpaceCostFunction2D, ceres::DYNAMIC, 3 >(
      new OccupiedSpaceCostFunction2D(scaling_factor, point_cloud, grid),// costfunctor
      point_cloud.size()); // 比固定残差维度的 多了一个参数
  
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
