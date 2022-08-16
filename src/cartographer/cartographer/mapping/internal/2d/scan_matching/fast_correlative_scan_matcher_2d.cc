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

#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

/************** SlidingWindowMaximum **************/

// A collection of values which can be added and later removed, and the maximum
// of the current values in the collection can be retrieved.
// All of it in (amortized) O(1).
// 滑动窗口算法
class SlidingWindowMaximum 
{
 public:
  // 向滑窗中添加值, 会将小于填入值的其他值删掉, 再将这个值放到最后
  void AddValue(const float value) 
  {
    // 如果添加的值大于滑窗中的最后一个值
    while (!non_ascending_maxima_.empty() && value > non_ascending_maxima_.back()) 
    {
      // 清空滑窗中的值
      non_ascending_maxima_.pop_back();
    }
    // 把输入的值添加到滑窗中，滑窗中的值是从小到大排序的
    non_ascending_maxima_.push_back(value);
  }

  // 删除值, 如果第一个值等于要删除的这个值, 则将这个值删掉
  void RemoveValue(const float value)
  {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    DCHECK(!non_ascending_maxima_.empty());
    DCHECK_LE(value, non_ascending_maxima_.front());
    if (value == non_ascending_maxima_.front())
    {
      non_ascending_maxima_.pop_front();
    }
  }

  // 获取最大值, 因为是按照顺序存储的, 第一个值是最大的
  float GetMaximum() const 
  {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    DCHECK_GT(non_ascending_maxima_.size(), 0);
    return non_ascending_maxima_.front();
  }

  void CheckIsEmpty() const { CHECK_EQ(non_ascending_maxima_.size(), 0); }

 private:
  // 前部当前滑动窗口的最大值。然后是该值首次出现后剩余窗口的最大值，依此类推。
  // 滑窗内的值
  std::deque<float> non_ascending_maxima_;
};

}  // namespace

/************** PrecomputationGrid2D **************/

proto::FastCorrelativeScanMatcherOptions2D
CreateFastCorrelativeScanMatcherOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::FastCorrelativeScanMatcherOptions2D options;
  options.set_linear_search_window(
      parameter_dictionary->GetDouble("linear_search_window"));
  options.set_angular_search_window(
      parameter_dictionary->GetDouble("angular_search_window"));
  options.set_branch_and_bound_depth(
      parameter_dictionary->GetInt("branch_and_bound_depth"));
  return options;
}

// 构造函数： 构造不同分辨率的地图
PrecomputationGrid2D::PrecomputationGrid2D(
    const Grid2D& grid,// 原始分辨率栅格地图 
    const CellLimits& limits,// 地图边界
    const int width,// width表示不同分辨率的栅格,最原始的地图是最细分辨率的栅格地图 width*width个原始地图栅格合成一个粗分辨率的栅格
    std::vector<float>* reusable_intermediate_grid)//  可以重复使用的中间栅格 用来保存最大值的一个中间变量
    : offset_(-width + 1, -width + 1),// 滑窗进入出去的空间
      wide_limits_(limits.num_x_cells + width - 1,
                   limits.num_y_cells + width - 1),
      min_score_(1.f - grid.GetMaxCorrespondenceCost()), // 0.1 min_score_
      max_score_(1.f - grid.GetMinCorrespondenceCost()), // 0.9 max_score_
      cells_(wide_limits_.num_x_cells * wide_limits_.num_y_cells)// 变大之后的新栅格
{
  CHECK_GE(width, 1);
  CHECK_GE(limits.num_x_cells, 1);
  CHECK_GE(limits.num_y_cells, 1);

  // x方向的栅格数 地图的一行的栅格数量
  const int stride = wide_limits_.num_x_cells;

  // First we compute the maximum probability for each (x0, y) achieved in the
  // span defined by x0 <= x < x0 + width.
  // 起别名
  std::vector<float>& intermediate = *reusable_intermediate_grid;
  //  中间变量的大小为加入滑动窗口后地图的大小
  intermediate.resize(wide_limits_.num_x_cells * limits.num_y_cells);
  

  // 枚举原始地图所有的y 即所有的行,把每一行分为一个一个长度为width的滑动窗口，求解出每个width段的最大值,保存到intermediate里
  // 对每一行从左到右横着做一次滑窗, 将滑窗后的地图放在intermediate(临时数据)中
  for (int y = 0; y != limits.num_y_cells; ++y)
  {
    // 滑窗
    SlidingWindowMaximum current_values;

    // 向尾部滑窗中添加一个值
    // 像素坐标系的原点位于左上角，(0, y)为左下角，1-空闲概率值 = 左下角占用概率值
    current_values.AddValue(1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(0, y))));// 每一行第一个栅格的值

    // step1 滑动窗口在x方向开始划入地图, 所以只进行 填入值
    // intermediate的索引：[x + width - 1 + y * stride]的范围是 [0 + y * stride, width-2 + y * stride]
    // grid的索引 x + width 的坐标范围是 [1, width-1]
    // 假设 width = 2，x = -1，x+1 = -width + 2 = 0
    for (int x = -width + 1; x != 0; ++x)
    {
      // 获取滑窗的最大值，赋值给临时数组
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      // -1+2 = 1
      if (x + width < limits.num_x_cells)
      {
        // 向滑窗中尾部添加坐标为(1,y)的栅格值
        current_values.AddValue(1.f - std::abs(grid.GetCorrespondenceCost(
                                          Eigen::Array2i(x + width, y))));
      }
    }

    // step2 滑动窗口已经完全在地图里了, 滑窗进行一入一出的操作
    // [x + width - 1 + y * stride] 的范围是 [width-1, limits.num_x_cells-2] 再加上 y * stride
    // grid的索引 x + width 的坐标范围是 [width, limits.num_x_cells-width-1]
    for (int x = 0; x < limits.num_x_cells - width; ++x) 
    {
      // 获取滑窗的最大值，赋值给临时数组
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      // 删除滑窗最左边的值：(0,y)、(1,y)
      current_values.RemoveValue(
          1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(x, y))));
      
      // 向滑窗中尾部添加值：(0+2,y)、(1+2,y)
      current_values.AddValue(1.f - std::abs(grid.GetCorrespondenceCost(
                                        Eigen::Array2i(x + width, y))));
    }
    // //滑动窗口要离开该行时,每往右滑动一次,删除一个值,直至完全滑出,滑动窗口里的值的个数为0
    // step3 滑动窗口正在划出, 一次减少一个值, 所以intermediate的宽度比grid多 width-1
    // x + width - 1 + y * stride 的范围是 [limits.num_x_cells-1, limits.num_x_cells+width-1] 再加上 y * stride
    // grid 的索引 x的范围是 [limits.num_x_cells-width, limits.num_x_cells-1]
    for (int x = std::max(limits.num_x_cells - width, 0); x != limits.num_x_cells; ++x) 
    {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      current_values.RemoveValue(
          1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(x, y))));
    }
    // 理论上, 滑窗走完地图的一行之后应该是空的, 经过 只入, 一出一入, 只出, 3个步骤
    current_values.CheckIsEmpty();
  }

  // 当遍历完每一行的时候,就实现了将width长的栅格(一个width包含多个栅格)值变成了一样(即多个栅格中的最大值),即行方向上的合并,接下来是列方向上的合并
  // 根据intermediate的值, 对每一列从下到上竖着再做一次滑窗, 这个才是真正的地图cells_
  for (int x = 0; x != wide_limits_.num_x_cells; ++x) 
  {
    SlidingWindowMaximum current_values;

    current_values.AddValue(intermediate[x]);
    // 滑入地图
    for (int y = -width + 1; y != 0; ++y) 
    {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      if (y + width < limits.num_y_cells) {
        current_values.AddValue(intermediate[x + (y + width) * stride]);
      }
    }
    
    for (int y = 0; y < limits.num_y_cells - width; ++y) 
    {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
      current_values.AddValue(intermediate[x + (y + width) * stride]);
    }
    // 滑出地图
    for (int y = std::max(limits.num_y_cells - width, 0);
         y != limits.num_y_cells; ++y) 
    {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
    }
    current_values.CheckIsEmpty();
  }
  // 当遍历完列后,就实现了行和列上的栅格的合并,即生成了width分辨率的地图,值得注意的是,原始地图一直是不变的,
  // 例如,width取2,4,8.生成widthorigin_resolution分辨率地图时,都是在原始地图上生成的,
  // 即4origin_resolution分辨率地图是在origin_resolution分辨率地图上生成的而不是在2*origin_resolution分辨率地图上生成的.

}



// 将概率值[0.1, 0.9]转成[0, 255]之间的灰度值
uint8 PrecomputationGrid2D::ComputeCellValue(const float probability) const 
{
  const int cell_value = common::RoundToInt(
      (probability - min_score_) * (255.f / (max_score_ - min_score_)));
  CHECK_GE(cell_value, 0);
  CHECK_LE(cell_value, 255);
  return cell_value;
}

// 构造多分辨率地图，分辨率 = 物理长度/网格个数
PrecomputationGridStack2D::PrecomputationGridStack2D(
    const Grid2D& grid,
    const proto::FastCorrelativeScanMatcherOptions2D& options) 
{

  CHECK_GE(options.branch_and_bound_depth(), 1);
  // 最粗的分辨率:是由分枝定界的深度决定的。
  // 分枝定界树的深度越小，生成的地图栅格个数越少，占用内存越少，但是匹配的时间会增加
  // param: branch_and_bound_depth 默认为7, 确定 最大的分辨率
  const int max_width = 1 << (options.branch_and_bound_depth() - 1); // 1左移动6位 == 64
  // 容器存放的是对象，初始化对象个数(容器容量) 即不同分辨率地图的个数
  precomputation_grids_.reserve(options.branch_and_bound_depth());
  
  // 创建保存地图栅格值的容器 可以重复使用的中间栅格
  std::vector<float> reusable_intermediate_grid;

  const CellLimits limits = grid.limits().cell_limits();

  // 经过滑窗后产生的栅格地图会变宽(滑入滑出), x方向最多会比原地图多max_width-1个格子
  reusable_intermediate_grid.reserve((limits.num_x_cells + max_width - 1) *
                                     limits.num_y_cells);

  // 分辨率逐渐变粗(网格数变少，即分母减少), i=0时就是默认分辨率0.05,一个格子得出一个栅格值
  // i=6时, width=64，64个格子合成一个格子得出一个栅格值
  for (int i = 0; i != options.branch_and_bound_depth(); ++i) 
  {
    // 构造各个不同分辨率的栅格地图 width表示不同分辨率的栅格,最原始的地图是最细分辨率的栅格地图
    // 1左移0等于1, 1左移动1等于2，1左移动2等于8....16、32、64，
    const int width = 1 << i;
    // notice: 构造不同分辨率的地图并保存 PrecomputationGrid2D原地构造函数初始化
    precomputation_grids_.emplace_back(grid, limits, width,
                                       &reusable_intermediate_grid);
  }
}

/************** FastCorrelativeScanMatcher2D **************/

// 构造函数
FastCorrelativeScanMatcher2D::FastCorrelativeScanMatcher2D(
    const Grid2D& grid,// 栅格地图的引用
    const proto::FastCorrelativeScanMatcherOptions2D& options)
    : options_(options),
      limits_(grid.limits()),
      // 初始化构建多分辨率地图
      precomputation_grid_stack_(
          absl::make_unique<PrecomputationGridStack2D>(grid, options)) {}

FastCorrelativeScanMatcher2D::~FastCorrelativeScanMatcher2D() {}

/**
 * @brief 进行局部搜索窗口的约束计算(对局部子图进行回环检测)
 * 
 * @param[in] initial_pose_estimate 先验位姿
 * @param[in] point_cloud 原点位于local坐标系原点处的点云
 * @param[in] min_score 最小阈值, 低于这个分数会返回失败
 * @param[out] score 匹配后的得分
 * @param[out] pose_estimate 匹配后得到的位姿
 * @return true 匹配成功, 反之匹配失败
 */

bool FastCorrelativeScanMatcher2D::Match(
    const transform::Rigid2d& initial_pose_estimate,// T_track2local
    const sensor::PointCloud& point_cloud, const float min_score,// 0.1 
    float* score,
    transform::Rigid2d* pose_estimate) const 
{
  // param: linear_search_window angular_search_window 
  // 构建线性搜索窗
  const SearchParameters search_parameters(options_.linear_search_window(),
                                           options_.angular_search_window(),
                                           point_cloud, limits_.resolution());
  
  // 返回匹配结果
  return MatchWithSearchParameters(search_parameters, initial_pose_estimate,
                                   point_cloud, min_score, score,
                                   pose_estimate);
}

/**
 * @brief 进行全局搜索窗口的约束计算(对整体子图进行回环检测)
 * 
 * @param[in] point_cloud 原点位于local坐标系原点处的点云
 * @param[in] min_score 最小阈值, 低于这个分数会返回失败
 * @param[out] score 匹配后的得分
 * @param[out] pose_estimate 匹配后得到的位姿
 * @return true 匹配成功, 反之匹配失败
 */
bool FastCorrelativeScanMatcher2D::MatchFullSubmap(
    const sensor::PointCloud& point_cloud, float min_score, float* score,
    transform::Rigid2d* pose_estimate) const 
{
  // Compute a search window around the center of the submap that includes it fully.
  // 将搜索窗口设置成 xy范围是1e6米, 角度范围是M_PI
  const SearchParameters search_parameters(
      1e6 * limits_.resolution(),  // 线性搜索窗口, 1e6 cells/direction.
      M_PI,  // 角度搜索窗口, 180 degrees in both directions = 360°.
      point_cloud, limits_.resolution());

  // 计算搜索窗口的中点(子图中点??)把这个中点作为搜索的起点，手动计算初始位姿
  const transform::Rigid2d center = transform::Rigid2d::Translation(
      limits_.max() - 0.5 * limits_.resolution() *
                          Eigen::Vector2d(limits_.cell_limits().num_y_cells,
                                          limits_.cell_limits().num_x_cells));

  // 返回匹配结果
  return MatchWithSearchParameters(search_parameters, center, point_cloud,
                                   min_score, score, pose_estimate);
}
/*
将点云转换到local坐标系下
生成按照不同角度旋转后的点云集合
将旋转后的点云集合按照机器人到local坐标系的平移量进行平移，获取平移后的点云在地图中的坐标索引
计算最低分辨率中的所有的候选解，返回值已更新为最优解在头部位置(从大到小，最低分辨率下)
基于最粗分辨率地图的候选解，使用多分辨率地图的分支定界搜索算法, 获取最细分辨率下的最优解
检查最优解的值, 如果大于指定阈值min_score就认为匹配成功，根据计算出的偏移量对初始位姿进行校准
否则认为不匹配返回失败


*/
bool FastCorrelativeScanMatcher2D::MatchWithSearchParameters(
    SearchParameters search_parameters,
    const transform::Rigid2d& initial_pose_estimate,// T_track2local 初始位姿
    const sensor::PointCloud& point_cloud, 
    float min_score,
    float* score,// out
    transform::Rigid2d* pose_estimate) const
{
  CHECK(score != nullptr);
  CHECK(pose_estimate != nullptr);

  // 初始姿态 R_track2local
  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  
  // step1 将原点处的点云先旋转到预测的方向上，R_track2local * P_gravity = P_local_R
  const sensor::PointCloud rotated_point_cloud = sensor::TransformPointCloud(
      point_cloud,
      transform::Rigid3f::Rotation(Eigen::AngleAxisf(
          initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ())));

  // step2 生成按照不同角度旋转后的点云集合：按照不同角度，每一个角度有一帧点云
  const std::vector<sensor::PointCloud> rotated_scans =
      GenerateRotatedScans(rotated_point_cloud, search_parameters);

  // step3 将旋转后的点云集合按照预测出的平移量进行 平移, 获取平移后的点云在地图中的坐标索引
  const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
      limits_, rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));// t_track2local
  
  // notice: 暴力匹配中没有
  // 缩小搜索窗口的大小, 计算每一帧点云在保证最后一个点能在地图范围内时的最大移动范围
  search_parameters.ShrinkToFit(discrete_scans, limits_.cell_limits());
  /*
  计算最低分辨率中的所有的候选解 最低分辨率是通过搜索树的层数、地图的分辨率计算出来的
  对于地图坐标系来说 最低分辨率=1<<h, h表示搜索树的总的层数
  这里不但对最低分辨率的所有候选解的得分进行了计算, 同时还按照从大到小排列
  */
  // 计算最低分辨率中的所有的候选解，返回最粗分辨率中的所有的候选解，最优解在头部位置(从大到小)
  const std::vector<Candidate2D> lowest_resolution_candidates =
      ComputeLowestResolutionCandidates(discrete_scans, search_parameters);
  
  // step4 基于最粗分辨率地图的候选解，使用多分辨率地图的分支定界搜索算法, 获取最细分辨率下的最优解
  const Candidate2D best_candidate = BranchAndBound(
      discrete_scans, search_parameters, lowest_resolution_candidates,// 最粗分辨率中的所有的候选解
      precomputation_grid_stack_->max_depth(), min_score); // param: max_depth
  
  // 检查最优解的值, 如果大于指定阈值min_score就认为匹配成功,否则认为不匹配返回失败
  if (best_candidate.score > min_score)
  {
    *score = best_candidate.score;
    // Step: 根据计算出的偏移量对初始位姿进行校准
    *pose_estimate = transform::Rigid2d(
        {initial_pose_estimate.translation().x() + best_candidate.x,
         initial_pose_estimate.translation().y() + best_candidate.y},
        initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
    return true;
  }
  return false;
}

// 生成最低分辨率层(栅格最粗)上的所有候选解, 并进行打分与排序
std::vector<Candidate2D>
FastCorrelativeScanMatcher2D::ComputeLowestResolutionCandidates(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters) const 
{

  // 生成最低分辨率层(栅格最粗)上的所有候选解
  std::vector<Candidate2D> lowest_resolution_candidates =
      GenerateLowestResolutionCandidates(search_parameters);

  // 计算每个候选解的得分, 按照匹配得分从大到小排序, 返回排列好的candidates 
  ScoreCandidates(
      precomputation_grid_stack_->Get(precomputation_grid_stack_->max_depth()),// 最粗的地图
      discrete_scans, search_parameters, &lowest_resolution_candidates);// lowest_resolution_candidates被更新为最优解在头部位置
  // lowest_resolution_candidates被更新为最优解在头部位置
  return lowest_resolution_candidates;
}

// 生成最低分辨率层(栅格最粗)上的所有候选解
std::vector<Candidate2D>
FastCorrelativeScanMatcher2D::GenerateLowestResolutionCandidates(
    const SearchParameters& search_parameters) const {
  const int linear_step_size = 1 << precomputation_grid_stack_->max_depth();
  int num_candidates = 0;
  // 遍历旋转后的每个点云
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) 
  {

    // X方向候选解的个数
    const int num_lowest_resolution_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + linear_step_size) /
        linear_step_size;

    // Y方向候选解的个数
    const int num_lowest_resolution_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + linear_step_size) /
        linear_step_size;

    // num_candidates 为最低分辨率这一层中所有候选解的总个数
    num_candidates += num_lowest_resolution_linear_x_candidates *
                      num_lowest_resolution_linear_y_candidates;
  }

  // 将所有候选解保存起来, 候选解的结构为（角度的索引, x偏移量, y偏移量, 搜索参数）
  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);

  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         x_index_offset += linear_step_size) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           y_index_offset += linear_step_size) {
        // 生成候选解, 存的是候选解与原始点云原点坐标间的偏移量
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

// 对所有的候选解进行评分并进行降序排序
void FastCorrelativeScanMatcher2D::ScoreCandidates(
    const PrecomputationGrid2D& precomputation_grid,
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate2D>* const candidates) const 
{
  // 遍历所有的候选解, 对每个候选解进行打分
  for (Candidate2D& candidate : *candidates) 
  {
    int sum = 0;
    // notice: 一帧点云的得分
    // xy_index 为这帧旋转后的点云上的每个点对应在地图上的栅格坐标
    for (const Eigen::Array2i& xy_index : discrete_scans[candidate.scan_index]) 
    {
      // 旋转后的点云的每个点的坐标加上这个可行解的X与Y的偏置, 即将点云进行平移
      const Eigen::Array2i proposed_xy_index(
          xy_index.x() + candidate.x_index_offset,
          xy_index.y() + candidate.y_index_offset);

      // 对平移后的点云的每个点 获取在precomputation_grid上对应的栅格值
      sum += precomputation_grid.GetValue(proposed_xy_index);
    }

    // 栅格值的和除以这个点云中点的个数, 作为这个候选解在这个 precomputation_grid 上的平均得分
    candidate.score = precomputation_grid.ToScore(
        sum / static_cast<float>(discrete_scans[candidate.scan_index].size()));
  }

  // 根据候选解的score, 对所有候选解进行降序排列，从大到小
  std::sort(candidates->begin(), candidates->end(),
            std::greater<Candidate2D>());
}

/**
 * @brief 基于多分辨率地图的分支定界搜索算法
 * 
 * @param[in] discrete_scans 多个点云的每个点在地图上的栅格坐标
 * @param[in] search_parameters 搜索配置参数
 * @param[in] candidates 候选解
 * @param[in] candidate_depth 搜索树的深度==6
 * @param[in] min_score 候选点最小得分 0.1 min_score用于减枝
 * @return Candidate2D 最优解
 */

Candidate2D FastCorrelativeScanMatcher2D::BranchAndBound(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    const std::vector<Candidate2D>& candidates, const int candidate_depth,
    float min_score) const
{
  
  /*
  这个函数是以递归调用的方式求解的
  首先给出了递归终止的条件, 就是如果到了第0层(到底了), 意味着我们搜索到了一个叶子节点.
  同时由于每次迭代过程我们都是对新扩展的候选点进行降序排列
  所以队首的这个叶子节点就是最优解, 直接返回即可
  */

  if (candidate_depth == 0)
  {
    // Return the best candidate.返回最优解
    return *candidates.begin();
  }

  // 然后创建一个临时的候选解, 并将得分设置为min_score，高分辨率最优解的初始化
  Candidate2D best_high_resolution_candidate(0, 0, 0, search_parameters);
  
  // 初始化为0.1，随着分辨率变细，后面被不断更新
  best_high_resolution_candidate.score = min_score;

  // 遍历最低分辨率所有的候选解
  for (const Candidate2D& candidate : candidates)
  {
    //  step1 剪枝 低于设置的阈值 或者 低于上一层的可行解的最高分 的可行解不进行继续分枝
    // 如果遇到一个候选点的分低于阈值, 那么后边的候选解的得分也会低于阈值,就可以直接跳出循环了
    if (candidate.score <= min_score)// 0.1
    {
      // 随着min_score的不断提高，其他候选解的得分小于min_score，被跳过，大量减小计算量
      break;
    }

    // 如果for循环能够继续运行, 说明当前候选点是一个更优的选择, 需要对其进行分枝
    std::vector<Candidate2D> higher_resolution_candidates;

    // 搜索步长减为上层的一半
    const int half_width = 1 << (candidate_depth - 1);

    // step2 分枝：对x、y偏移进行遍历, 求出当前candidate的四个子节点候选解 notice: 
    for (int x_offset : {0, half_width})
    { // 只能取0和half_width

      // 如果超过了界限, 就跳过
      if (candidate.x_index_offset + x_offset >
          search_parameters.linear_bounds[candidate.scan_index].max_x) 
      {
        break;
      }
      // 2² = 4
      for (int y_offset : {0, half_width})
      {
        if (candidate.y_index_offset + y_offset >
            search_parameters.linear_bounds[candidate.scan_index].max_y) 
        {
          break;
        }
        // 候选者依次推进来, 一共4个,可以看出, 分枝定界方法的分枝是向右下角的四个子节点进行分枝
        higher_resolution_candidates.emplace_back(
            candidate.scan_index, candidate.x_index_offset + x_offset,
            candidate.y_index_offset + y_offset, search_parameters);
      }
    }

    // 对新生成的4个候选解进行打分与排序, 同一个点云, 不同地图
    ScoreCandidates(precomputation_grid_stack_->Get(candidate_depth - 1),// 最粗分辨率地图的下一层地图
                    discrete_scans, search_parameters,
                    &higher_resolution_candidates);
    /*
    递归调用BranchAndBound对新生成的higher_resolution_candidates进行搜索 
    先对其分数最高的节点继续进行分支, 直到最底层(最底层的最高分作为界限), 
    然后再返回倒数第二层再进行迭代
    如果倒数第二层的最高分没有上一个的最底层（叶子层）的分数高, 则跳过, 
    否则继续向下进行分支与评分
    */

    // step3 定界，深度优先的递归调用
    // notice: 之后通过递归调用发现了更优的解都将通过std::max函数来更新已知的最优解.
    best_high_resolution_candidate = std::max(
        best_high_resolution_candidate,
        BranchAndBound(discrete_scans, search_parameters,
                       higher_resolution_candidates, 
                       candidate_depth - 1,// 层数不断往下，直到最细分辨率地图
                       best_high_resolution_candidate.score));// min_score用于减枝
  }

  // 返回最高分辨率的最优解
  return best_high_resolution_candidate;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
