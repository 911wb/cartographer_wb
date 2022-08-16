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

#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/internal/2d/tsdf_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

// 计算点云在指定像素坐标位置下与TSDF2D地图匹配的得分
float ComputeCandidateScore(const TSDF2D& tsdf,
                            const DiscreteScan2D& discrete_scan,
                            int x_index_offset, int y_index_offset) {
  float candidate_score = 0.f;
  float summed_weight = 0.f;
  for (const Eigen::Array2i& xy_index : discrete_scan) {
    const Eigen::Array2i proposed_xy_index(xy_index.x() + x_index_offset,
                                           xy_index.y() + y_index_offset);
    const std::pair<float, float> tsd_and_weight =
        tsdf.GetTSDAndWeight(proposed_xy_index);
    const float normalized_tsd_score =
        (tsdf.GetMaxCorrespondenceCost() - std::abs(tsd_and_weight.first)) /
        tsdf.GetMaxCorrespondenceCost();
    const float weight = tsd_and_weight.second;
    candidate_score += normalized_tsd_score * weight;
    summed_weight += weight;
  }
  if (summed_weight == 0.f) return 0.f;
  candidate_score /= summed_weight;
  CHECK_GE(candidate_score, 0.f);
  return candidate_score;
}

// 计算点云在指定像素坐标位置下与ProbabilityGrid地图匹配的得分
float ComputeCandidateScore(const ProbabilityGrid& probability_grid,// 概率栅格地图
                            const DiscreteScan2D& discrete_scan,// 旋转后点云的像素坐标
                            int x_index_offset, int y_index_offset)// 点云平移量
{
  float candidate_score = 0.f;

  // 遍历每一个旋转后的一帧点云，将它们进行平移
  for (const Eigen::Array2i& xy_index : discrete_scan) 
  {
    // 对每个点都加上像素坐标的offset, 相当于对一个点云进行平移，得到平移后的像素坐标
    const Eigen::Array2i proposed_xy_index(xy_index.x() + x_index_offset,
                                           xy_index.y() + y_index_offset);
    // 点云像素坐标对应的占用的概率值
    const float probability =
        probability_grid.GetProbability(proposed_xy_index);
    // 以概率值为得分
    candidate_score += probability;
  }

  // 计算当前这一帧点云的平均得分：0~1
  candidate_score /= static_cast<float>(discrete_scan.size());
  CHECK_GT(candidate_score, 0.f);
  return candidate_score;
}

}  // namespace

// 初始化成员变量
RealTimeCorrelativeScanMatcher2D::RealTimeCorrelativeScanMatcher2D(
    const proto::RealTimeCorrelativeScanMatcherOptions& options)
    : options_(options) {}

// 生成所有的候选解
std::vector<Candidate2D>
RealTimeCorrelativeScanMatcher2D::GenerateExhaustiveSearchCandidates(
    const SearchParameters& search_parameters) const 
{

  int num_candidates = 0;
  // 遍历旋转后的点云，
  // 遍历所有旋转角度，计算候选解的总个数
  for (int scan_index = 0; scan_index != search_parameters.num_scans; ++scan_index)
  {
    // 这一个角度对应的x的可能性
    const int num_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + 1);

    const int num_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + 1);

    // 所有角度下、所有x方向上、所有y方向上，候选解的总个数
    num_candidates += num_linear_x_candidates * num_linear_y_candidates;
  }
  // 保存候选解的容器
  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);

  // 生成候选解, 候选解是由像素坐标的偏差组成的 o(n^3)
  // 分别遍历角度Θ、遍历x、遍历y，先遍历角度是应为角度的个数较小 2^3 < 3^2
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         ++x_index_offset) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           ++y_index_offset) 
      {
        // 原地构造
        candidates.emplace_back(scan_index,// 第几个角度
                                x_index_offset,// Δx
                                y_index_offset,// Δy
                                search_parameters);// ΔR
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

/**
 * @brief 相关性扫描匹配 - 暴力匹配计算量很大
 * 
 * @param[in] initial_pose_estimate 先验位姿
 * @param[in] point_cloud 经过重力垂直，体素滤波后的点云(local坐标系下)
 * @param[in] grid 用于匹配的栅格地图
 * @param[out] pose_estimate 校正后的后验位姿
 * @return double 匹配得分
 */

/*
匹配思路其实十分简单，就是将历史点云数据构建为一个参考栅格地图，再将当前激光雷达摆放在栅格地图上（遍历位置与角度），
哪一个摆放位姿下的当前点云与参考栅格地图匹配得分最高，那个位姿就是估计的位姿。
*/
double RealTimeCorrelativeScanMatcher2D::Match(
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, 
    const Grid2D& grid,
    transform::Rigid2d* pose_estimate) const 
{
  
  CHECK(pose_estimate != nullptr);

  // 机器人先验位姿的姿态
  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  
  // 把激光数据旋转到和世界坐标系平行的坐标系中，旋转是因为把激光数据旋转会0度。
  // 因为接下来的搜索中，我们要对角度进行离散化搜索，从0度开始进行搜索
  // step1 将原点处的点云先旋转到预测的方向上  R_track2local * P_gravity = P_local_R
  const sensor::PointCloud rotated_point_cloud = sensor::TransformPointCloud(
      point_cloud, transform::Rigid3f::Rotation(Eigen::AngleAxisf(
          initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ())));

  // 根据配置参数初始化 搜索窗口
  const SearchParameters search_parameters(
      options_.linear_search_window(), options_.angular_search_window(),
      rotated_point_cloud, grid.limits().resolution());

  // step2 生成按照不同角度，每一个角度有一帧点云的点云集合
  const std::vector<sensor::PointCloud> rotated_scans =
      GenerateRotatedScans(rotated_point_cloud, search_parameters);
  
  // step3 将旋转后的点云集合按照预测出的平移量进行平移, 获取P_local在栅格地图中的索引
  const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
      grid.limits(), rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));
  
  // step4 生成所有的候选解
  std::vector<Candidate2D> candidates =
      GenerateExhaustiveSearchCandidates(search_parameters);
  
  // step5 计算所有候选解的加权得分，更新候选解candidates
  ScoreCandidates(grid, discrete_scans, search_parameters, &candidates);

  // step6 从候选解中获取最优解(得分最大的解)
  const Candidate2D& best_candidate =
      *std::max_element(candidates.begin(), candidates.end());
  
  // step7 将计算出的偏差量加上原始先验位姿，暴力匹配校准后的位姿 = T_new = T_old + ΔT
  // 当前先验位姿下的点云，与障碍物之间并没有重合，有角度和xy方向上的偏差，需要调整机器人位姿使得偏差尽量小
  *pose_estimate = transform::Rigid2d(
      {initial_pose_estimate.translation().x() + best_candidate.x,
       initial_pose_estimate.translation().y() + best_candidate.y},
      initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));

  // 返回最优解的分数
  return best_candidate.score;
}

// 计算所有候选解的加权得分
// 入参：栅格地图、旋转后的点云的像素坐标、搜索参数
void RealTimeCorrelativeScanMatcher2D::ScoreCandidates(
    const Grid2D& grid, const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate2D>* const candidates) const 
{
  // 遍历所有候选解，一个候选解对应一帧点云
  for (Candidate2D& candidate : *candidates) 
  {
    switch (grid.GetGridType()) 
    {
      case GridType::PROBABILITY_GRID:
        candidate.score = ComputeCandidateScore(
            static_cast<const ProbabilityGrid&>(grid),
            discrete_scans[candidate.scan_index], candidate.x_index_offset,
            candidate.y_index_offset);
        break;
      case GridType::TSDF:
        candidate.score = ComputeCandidateScore(
            static_cast<const TSDF2D&>(grid),
            discrete_scans[candidate.scan_index], candidate.x_index_offset,
            candidate.y_index_offset);
        break;
    }
    
    // 对得分进行加权 candidate.score = candidate.score * e^(-d*w + R*w)
    candidate.score *=
        std::exp(-common::Pow2(std::hypot(candidate.x, candidate.y) *
                                   options_.translation_delta_cost_weight() +
                               std::abs(candidate.orientation) *
                                   options_.rotation_delta_cost_weight()));
  }
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
