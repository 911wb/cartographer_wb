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

#include "cartographer/mapping/2d/submap_2d.h"

#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>

#include "Eigen/Geometry"
#include "absl/memory/memory.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/internal/2d/tsdf_range_data_inserter_2d.h"
#include "cartographer/mapping/range_data_inserter_interface.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// submap2d的参数配置配置
proto::SubmapsOptions2D CreateSubmapsOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) 
{
  proto::SubmapsOptions2D options;
  options.set_num_range_data(
      parameter_dictionary->GetNonNegativeInt("num_range_data"));
  *options.mutable_grid_options_2d() = CreateGridOptions2D(
      parameter_dictionary->GetDictionary("grid_options_2d").get());
  *options.mutable_range_data_inserter_options() =
      CreateRangeDataInserterOptions(
          parameter_dictionary->GetDictionary("range_data_inserter").get());

  bool valid_range_data_inserter_grid_combination = false;
  // 地图类型
  const proto::GridOptions2D_GridType& grid_type =
      options.grid_options_2d().grid_type();
  // 将scan写成地图的方式
  const proto::RangeDataInserterOptions_RangeDataInserterType&
      range_data_inserter_type =
          options.range_data_inserter_options().range_data_inserter_type();
  if (grid_type == proto::GridOptions2D::PROBABILITY_GRID &&
      range_data_inserter_type ==
          proto::RangeDataInserterOptions::PROBABILITY_GRID_INSERTER_2D) {
    valid_range_data_inserter_grid_combination = true;
  }
  if (grid_type == proto::GridOptions2D::TSDF &&
      range_data_inserter_type ==
          proto::RangeDataInserterOptions::TSDF_INSERTER_2D) {
    valid_range_data_inserter_grid_combination = true;
  }
  CHECK(valid_range_data_inserter_grid_combination)
      << "Invalid combination grid_type " << grid_type
      << " with range_data_inserter_type " << range_data_inserter_type;
  CHECK_GT(options.num_range_data(), 0);
  return options;
}

/**
 * @brief 构造函数
 * 
 * @param[in] origin Submap2D的原点,保存在Submap类里
 * @param[in] grid 地图数据的指针
 * @param[in] conversion_tables 地图数据的转换表
 */
Submap2D::Submap2D(const Eigen::Vector2f& origin, std::unique_ptr<Grid2D> grid,
                   ValueConversionTables* conversion_tables)
      // t_track2local 作为子图的原点坐标 local_pose_
    : Submap(transform::Rigid3d::Translation(Eigen::Vector3d(origin.x(), origin.y(), 0.))),
      conversion_tables_(conversion_tables)
{
  grid_ = std::move(grid);
}

// 根据proto::Submap格式的数据生成Submap2D
Submap2D::Submap2D(const proto::Submap2D& proto,
                   ValueConversionTables* conversion_tables)
    : Submap(transform::ToRigid3(proto.local_pose())),
      conversion_tables_(conversion_tables)
{
  if (proto.has_grid()) 
  {
    if (proto.grid().has_probability_grid_2d()) 
    {
      grid_ =
          absl::make_unique<ProbabilityGrid>(proto.grid(), conversion_tables_);
    }
    else if (proto.grid().has_tsdf_2d()) 
    {
      grid_ = absl::make_unique<TSDF2D>(proto.grid(), conversion_tables_);
    }
    else
    {
      LOG(FATAL) << "proto::Submap2D has grid with unknown type.";
    }
  }
  set_num_range_data(proto.num_range_data());
  set_insertion_finished(proto.finished());
}

// 根据mapping::Submap2D生成proto::Submap格式的数据
proto::Submap Submap2D::ToProto(const bool include_grid_data) const 
{
  proto::Submap proto;
  auto* const submap_2d = proto.mutable_submap_2d();
  *submap_2d->mutable_local_pose() = transform::ToProto(local_pose());
  submap_2d->set_num_range_data(num_range_data());
  submap_2d->set_finished(insertion_finished());
  if (include_grid_data)
  {
    CHECK(grid_);
    *submap_2d->mutable_grid() = grid_->ToProto();
  }
  return proto;
}

// 根据proto::Submap格式的数据更新地图
void Submap2D::UpdateFromProto(const proto::Submap& proto) {
  CHECK(proto.has_submap_2d());
  const auto& submap_2d = proto.submap_2d();
  set_num_range_data(submap_2d.num_range_data());
  set_insertion_finished(submap_2d.finished());
  if (proto.submap_2d().has_grid()) {
    if (proto.submap_2d().grid().has_probability_grid_2d()) {
      grid_ = absl::make_unique<ProbabilityGrid>(proto.submap_2d().grid(),
                                                 conversion_tables_);
    } else if (proto.submap_2d().grid().has_tsdf_2d()) {
      grid_ = absl::make_unique<TSDF2D>(proto.submap_2d().grid(),
                                        conversion_tables_);
    } else {
      LOG(FATAL) << "proto::Submap2D has grid with unknown type.";
    }
  }
}

/**
 * @brief 将地图进行压缩, 放入response
 * 
 * @param[out] response 压缩后的地图数据
 */
void Submap2D::ToResponseProto(
    const transform::Rigid3d&,
    proto::SubmapQuery::Response* const response) const 
{
  if (!grid_) 
    return;
  response->set_submap_version(num_range_data());

  // notice: const在*后边, 指针指向的地址不能变,而内存单元中的内容可变
  proto::SubmapQuery::Response::SubmapTexture* const texture =
      response->add_textures();
      
  // 填充压缩后的数据
  grid()->DrawToSubmapTexture(texture, local_pose());
}

// 将雷达数据写到栅格地图中
void Submap2D::InsertRangeData(
    const sensor::RangeData& range_data,
    const RangeDataInserterInterface* range_data_inserter) 
{
  CHECK(grid_);
  CHECK(!insertion_finished());

  //  将一帧点云数据写到栅格地图中，传入之前创建的100*100的栅格地图
  range_data_inserter->Insert(range_data, grid_.get());
  // 插入到子图中雷达数据的个数 加1
  set_num_range_data(num_range_data() + 1);
}

// 将子图标记为完成状态
void Submap2D::Finish()
{
  CHECK(grid_);
  CHECK(!insertion_finished());
  // 根据bounding_box对栅格地图进行裁剪到正好包含所有点云
  grid_ = grid_->ComputeCroppedGrid();
  // 将子图标记为完成状态
  set_insertion_finished(true);
}

/********** ActiveSubmaps2D *****************/

// ActiveSubmaps2D构造函数
ActiveSubmaps2D::ActiveSubmaps2D(const proto::SubmapsOptions2D& options)
    : options_(options), range_data_inserter_(CreateRangeDataInserter()) {}

std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::submaps() const 
{
  // 返回指向 Submap2D 的 shared_ptr指针 的vector
  // 返回一个容器，该容器的所有智能指针都指向Submap2D，容器中存放的是指向子图的指针(返回所有子图)
  return std::vector<std::shared_ptr<const Submap2D>>(submaps_.begin(), submaps_.end());
}

/*
将点云数据写入到submap中的步骤
第一帧点云进来submaps_为空
  1. 以当前机器人t_track2local为子图原点创建子图边界:最大值(x+50个栅格对应的物理长度, y + 50个栅格对应的物理长度)，初始为未知，
    整个地图的大小为100*100，以t_track2local为中心
  2.bounding_box从原点往外扩展，使得bounding_box包括所有returns和misses的点云
  3.计算bounding_box最大最小物理坐标点的对应像素坐标，判断该像素坐标是否在地图坐标系内，如果像素坐标是 不在 栅格地图内部，则对地图进行扩大
    中心点不变, 子图向四周扩大两倍，复制到将老地图的栅格值新地图上，但新地图内老地图外的栅格的值仍是0:未知
  4.更新子图栅格值：origin -> hits ,origin -> misses
  5.当插入子图的数据为180帧时，一张子图建立完毕。

开始子图容器为空新建一个子图，使用当前传入的t_track2local为原点
 ____
|    | submap0 点云帧数 = 1
|____|
不断写入一帧点云......直到
 ____
|    | submap0 点云帧数 = 90
|____|

新建一个子图，使用当前传入的t_track2local为子图原点
 ____
|    | submap1 点云帧数 = 1
|____|
不断写入一帧点云......直到
submap0 点云帧数 = 180，submap1 点云帧数 = 90
删除submap0，新建submap2，使用当前传入的t_track2local为子图原点
...不断循环

*/
// 将点云数据写入到submap中
std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::InsertRangeData(
    const sensor::RangeData& range_data) 
{
  // 如果第二个子图插入节点的数据等于num_range_data时,就新建个子图,因为这时第一个子图应该已经处于完成状态了
  if (submaps_.empty() ||
      submaps_.back()->num_range_data() == options_.num_range_data())// num_range_data = 90： 插入到子图中点云数据的帧数
  {
    // 添加子图
    AddSubmap(range_data.origin.head<2>());
  }

  // 将一帧雷达数据同时写入2个子图中：每两张子图，会有一半的重合，匹配时两个栅格有交集，保证匹配得上
  for (auto& submap : submaps_)
  {
    // 取出以t_track2local为原点的栅格地图，将点云数据写到栅格地图中，插入到子图中雷达数据的 帧数 加1
    submap->InsertRangeData(range_data, range_data_inserter_.get());
  }

  // 第一个子图的节点数量等于2倍的num_range_data时, 第二个子图节点数量应该等于num_range_data
  // 当插入子图的数据为180帧时，一张子图建立完毕。
  if (submaps_.front()->num_range_data() == 2 * options_.num_range_data())// sub_1 = 2*90, sub_2 = 90
  {
    // 该子图标记为完成状态，不再更新，一张子图已经建立完毕
    submaps_.front()->Finish();
  }

  // 返回一个容器，该容器的所有智能指针都指向Submap2D，容器中存放的是指向子图的指针(返回一个或两个子图或不完整子图)
  return submaps();

}

// 创建地图数据写入器
std::unique_ptr<RangeDataInserterInterface>
ActiveSubmaps2D::CreateRangeDataInserter() 
{
  switch (options_.range_data_inserter_options().range_data_inserter_type()) 
  {
    // 创建概率栅格地图的写入器
    case proto::RangeDataInserterOptions::PROBABILITY_GRID_INSERTER_2D:
      // 返回指向ProbabilityG的智能指针
      return absl::make_unique<ProbabilityGridRangeDataInserter2D>(
          options_.range_data_inserter_options()
              .probability_grid_range_data_inserter_options_2d());

    // tsdf地图的写入器
    case proto::RangeDataInserterOptions::TSDF_INSERTER_2D:
      return absl::make_unique<TSDFRangeDataInserter2D>(
          options_.range_data_inserter_options()
              .tsdf_range_data_inserter_options_2d());
    default:
      LOG(FATAL) << "Unknown RangeDataInserterType.";
  }
}

// 以当前机器人t_track2local为子图原点创建子图边界:最大值(x+50个栅格对应的物理长度, y + 50个栅格对应的物理长度)，初始为未知
std::unique_ptr<GridInterface> ActiveSubmaps2D::CreateGrid(
    const Eigen::Vector2f& origin)
{
  // 地图初始大小,100个栅格
  constexpr int kInitialSubmapSize = 100;

  // param: grid_options_2d.resolution = 0.05，室外：0.1，室内0.05
  float resolution = options_.grid_options_2d().resolution(); 

  switch (options_.grid_options_2d().grid_type())
  {
    /*       
    cartographer的地图物理坐标系 
              ^ x
              |
              |
     y <------0 
    origin位于地图的中间，origin向x方向移动50格对应的物理长度，
    向y方向移动50格对应的物理长度，得到最大值
    */
    case proto::GridOptions2D::PROBABILITY_GRID:
      // 返回概率栅格地图，ProbabilityGrid构造函数初始化：栅格值栅格数、地图边界
      return absl::make_unique<ProbabilityGrid>(
          // 构造函数初始化
          MapLimits(resolution,// 物理长度/栅格数
                    // 以当前机器人位置为中心，origin + (x+50个栅格对应的物理长度, y + 50个栅格对应的物理长度)为子图最大值
                    origin.cast<double>() + 0.5 * kInitialSubmapSize *
                    resolution * Eigen::Vector2d::Ones(),// 50个栅格对应的物理长度
                    // 构造函数初始化：x方向的栅格数 = y方向的栅格数 = 100
                    CellLimits(kInitialSubmapSize, kInitialSubmapSize)),
          &conversion_tables_);

    // tsdf地图
    case proto::GridOptions2D::TSDF:
      return absl::make_unique<TSDF2D>(
          MapLimits(resolution,
                    origin.cast<double>() + 0.5 * kInitialSubmapSize *
                                                resolution *
                                                Eigen::Vector2d::Ones(),
                    CellLimits(kInitialSubmapSize, kInitialSubmapSize)),
          options_.range_data_inserter_options()
              .tsdf_range_data_inserter_options_2d()
              .truncation_distance(),               // 0.3
          options_.range_data_inserter_options()
              .tsdf_range_data_inserter_options_2d()
              .maximum_weight(),                    // 10.0
          &conversion_tables_);
    default:
      LOG(FATAL) << "Unknown GridType.";
  }

}

// 新增一个子图,根据子图个数判断是否删掉第一个子图
void ActiveSubmaps2D::AddSubmap(const Eigen::Vector2f& origin) 
{
  // 调用AddSubmap时第一个子图一定是完成状态,所以子图数为2时就可以删掉第一个子图了
  if (submaps_.size() >= 2)
  {
    // This will crop the finished Submap before inserting a new Submap to
    // reduce peak memory usage a bit.
    CHECK(submaps_.front()->insertion_finished());

    // 删掉第一个子图的指针，计数器-1
    submaps_.erase(submaps_.begin());
  }

  // 调用构造函数新建一个指向Submap2D的智能指针, 存入容器中
  submaps_.push_back(absl::make_unique<Submap2D>(
      origin,// t_track2local
      // 以当前机器人t_track2local为子图原点创建子图边界:最大值(x+50个栅格对应的物理长度, y + 50个栅格对应的物理长度)，初始为未知
      std::unique_ptr<Grid2D>(static_cast<Grid2D*>(CreateGrid(origin).release())),// grid
      &conversion_tables_));
}

}  // namespace mapping
}  // namespace cartographer
