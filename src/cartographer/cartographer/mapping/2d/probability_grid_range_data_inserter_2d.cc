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

#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"

#include <cstdlib>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/mapping/internal/2d/ray_to_pixel_mask.h"
#include "cartographer/mapping/probability_values.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

// Factor for subpixel accuracy of start and end point for ray casts.
constexpr int kSubpixelScale = 1000;

// 根据点云的bounding box, 看是否需要对地图进行扩张
void GrowAsNeeded(const sensor::RangeData& range_data,
                  ProbabilityGrid* const probability_grid)
{

  // 点云围绕的中心作为bounding_box 2D框中心??
  // t_track2local
  Eigen::AlignedBox2f bounding_box(range_data.origin.head<2>());

  // 在边界框周围填充，以避免单元格边界出现数值问题。
  constexpr float kPadding = 1e-6f;

  // bounding_box从原点往外扩展，使得bounding_box包括所有returns和misses的点云
  for (const sensor::RangefinderPoint& hit : range_data.returns) 
  {
    // 扩展区域的边界，使得该区域包含输入的点
    bounding_box.extend(hit.position.head<2>());
  }
  for (const sensor::RangefinderPoint& miss : range_data.misses) 
  {
    // 扩展区域的边界，使得该区域包含输入的点
    bounding_box.extend(miss.position.head<2>());
  }

  // 根据坐标决定是否对地图进行扩大
  probability_grid->GrowLimits(bounding_box.min() -
                               kPadding * Eigen::Vector2f::Ones());
  probability_grid->GrowLimits(bounding_box.max() +
                               kPadding * Eigen::Vector2f::Ones());
}

/**
 * @brief 根据雷达点对栅格地图进行更新
 * 
 * @param[in] range_data 所有雷达点云
 * @param[in] hit_table 更新占用栅格时的查找表
 * @param[in] miss_table 更新空闲栅格时的查找表
 * @param[in] insert_free_space 是否添加空闲区域
 * @param[in] probability_grid 栅格地图
 */
void CastRays(const sensor::RangeData& range_data,
              const std::vector<uint16>& hit_table,
              const std::vector<uint16>& miss_table,
              const bool insert_free_space, ProbabilityGrid* probability_grid)// 之前建立的grid：origin+(x+50, y+50)
{
  // 根据点云数据调整地图范围
  // 根据点云数据得到一张子图，t_track2local为中心，最大点云坐标为边界的栅格地图，一开始地图中的栅格值初始化为0，后面只有扩展区域栅格值初始化为0
  GrowAsNeeded(range_data, probability_grid);

  // 扩大地图的后边界
  const MapLimits& limits = probability_grid->limits();
  // 单位物理长度被分为更多栅格，物理长度/栅格数×1000
  const double superscaled_resolution = limits.resolution() / kSubpixelScale;
  // 创建更细密的坐标系，以获取更精确的坐标值
  const MapLimits superscaled_limits(
      superscaled_resolution, limits.max(),
      CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,// x方向栅格数 × 1000
                 limits.cell_limits().num_y_cells * kSubpixelScale));// y方向栅格数 × 1000
  
  // t_track2local(物理坐标)在地图中的像素坐标, 作为画线的起始坐标
  const Eigen::Array2i begin =
      superscaled_limits.GetCellIndex(range_data.origin.head<2>());
  
  // Compute and add the end points.
  std::vector<Eigen::Array2i> ends;

  ends.reserve(range_data.returns.size());

  // 遍历所有返回的(击中的)激光点
  for (const sensor::RangefinderPoint& hit : range_data.returns) 
  {
    // 获取hit点在更精细坐标系中，击中的激光点的像素坐标，并存入ends作为 画线 的终止点坐标
    ends.push_back(superscaled_limits.GetCellIndex(hit.position.head<2>()));

    // 通过查找表，更新hit点的栅格值
    probability_grid->ApplyLookupTable(ends.back() / kSubpixelScale, hit_table);
  }
  
  // 如果不插入free空间就可以结束了
  if (!insert_free_space)
  {
    return;
  }

  // 遍历所有击中的点  从原点指向击中的点，前面击中的点的栅格值已经更新了，接下来更新原点到hit点间的栅格值
  for (const Eigen::Array2i& end : ends)
  {
    // 从原点到hit点之间，获取未击中的栅格坐标(像素坐标) ，这条线经过未击中的栅格坐标
    // 已知原点和hit点（1000倍细分），求该两点连线经过的栅格：Bresenham算法
    std::vector<Eigen::Array2i> ray =
        RayToPixelMask(begin, end, kSubpixelScale);

    // 遍历从原点到hit点之间的空闲点
    for (const Eigen::Array2i& cell_index : ray)
    {
      // 更新这些点的空闲状态的栅格值
      probability_grid->ApplyLookupTable(cell_index, miss_table);
    }
  }

  // 最后，更新超过了使用距离的点的栅格值
  for (const sensor::RangefinderPoint& missing_echo : range_data.misses)
  {
    // 已知原点和misses点（1000倍细分），求该两点连线经过的栅格：Bresenham算法
    std::vector<Eigen::Array2i> ray = RayToPixelMask(
        begin, superscaled_limits.GetCellIndex(missing_echo.position.head<2>()),
        kSubpixelScale);
    // 遍历从原点到misses点之间的空闲点
    for (const Eigen::Array2i& cell_index : ray)
    {
      // 更新这些点的空闲状态的栅格值
      probability_grid->ApplyLookupTable(cell_index, miss_table);
    }
  }
}
}  // namespace

proto::ProbabilityGridRangeDataInserterOptions2D
CreateProbabilityGridRangeDataInserterOptions2D(
    common::LuaParameterDictionary* parameter_dictionary) {
  proto::ProbabilityGridRangeDataInserterOptions2D options;
  options.set_hit_probability(
      parameter_dictionary->GetDouble("hit_probability"));
  options.set_miss_probability(
      parameter_dictionary->GetDouble("miss_probability"));
  options.set_insert_free_space(
      parameter_dictionary->HasKey("insert_free_space")
          ? parameter_dictionary->GetBool("insert_free_space")
          : true);
  CHECK_GT(options.hit_probability(), 0.5);
  CHECK_LT(options.miss_probability(), 0.5);
  return options;
}

// 写入器的构造, 制造了2个查找表
ProbabilityGridRangeDataInserter2D::ProbabilityGridRangeDataInserter2D(
    const proto::ProbabilityGridRangeDataInserterOptions2D& options)
    : options_(options),
      // 生成更新占用栅格时的查找表 // param: hit_probability
      hit_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
          Odds(options.hit_probability()))),    // 0.55 --> P/(1-P) = 0.55/0.45 = 1.22
      // 生成更新空闲栅格时的查找表 // param: miss_probability
      miss_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
          Odds(options.miss_probability()))) {} // 0.49 --> P/(1-P) = 0.49/0.51 = 0.96

/**
 * @brief 将点云写入栅格地图
 * 
 * @param[in] range_data 要写入地图的点云
 * @param[in] grid 栅格地图
 */
void ProbabilityGridRangeDataInserter2D::Insert(
    const sensor::RangeData& range_data, GridInterface* const grid) const 
{

  ProbabilityGrid* const probability_grid = static_cast<ProbabilityGrid*>(grid);

  CHECK(probability_grid != nullptr);
  // By not finishing the update after hits are inserted, we give hits priority
  // (i.e. no hits will be ignored because of a miss in the same cell).
  // param: insert_free_space
  
  CastRays(range_data, hit_table_, miss_table_, options_.insert_free_space(), probability_grid);
  
  probability_grid->FinishUpdate();
}

}  // namespace mapping
}  // namespace cartographer
