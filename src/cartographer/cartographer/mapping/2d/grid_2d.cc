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
#include "cartographer/mapping/2d/grid_2d.h"

namespace cartographer {
namespace mapping {
namespace {

float MinCorrespondenceCostFromProto(const proto::Grid2D& proto) {
  if (proto.min_correspondence_cost() == 0.f &&
      proto.max_correspondence_cost() == 0.f) {
    LOG(WARNING) << "proto::Grid2D: min_correspondence_cost "
                    "is initialized with 0 indicating an older version of the "
                    "protobuf format. Loading default values.";
    return kMinCorrespondenceCost;
  } else {
    return proto.min_correspondence_cost();
  }
}

float MaxCorrespondenceCostFromProto(const proto::Grid2D& proto) {
  if (proto.min_correspondence_cost() == 0.f &&
      proto.max_correspondence_cost() == 0.f) {
    LOG(WARNING) << "proto::Grid2D: max_correspondence_cost "
                    "is initialized with 0 indicating an older version of the "
                    "protobuf format. Loading default values.";
    return kMaxCorrespondenceCost;
  } else {
    return proto.max_correspondence_cost();
  }
}
}  // namespace

// 获取参数配置
proto::GridOptions2D CreateGridOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary)
{
  proto::GridOptions2D options;
  const std::string grid_type_string =
      parameter_dictionary->GetString("grid_type");
  proto::GridOptions2D_GridType grid_type;
  CHECK(proto::GridOptions2D_GridType_Parse(grid_type_string, &grid_type))
      << "Unknown GridOptions2D_GridType kind: " << grid_type_string;
  options.set_grid_type(grid_type);
  options.set_resolution(parameter_dictionary->GetDouble("resolution"));
  return options;
}

/**
 * @brief 构造函数
 * 
 * @param[in] limits 地图坐标信息
 * @param[in] min_correspondence_cost 最小correspondence_cost 0.1
 * @param[in] max_correspondence_cost 最大correspondence_cost 0.9
 * @param[in] conversion_tables 传入的转换表指针
 */

Grid2D::Grid2D(const MapLimits& limits, float min_correspondence_cost,
               float max_correspondence_cost,
               ValueConversionTables* conversion_tables)
    : limits_(limits),
      correspondence_cost_cells_(// 容器的容量为50*50，初始值为0
          // 将整个地图分为 x*y 个栅格，每个栅格的值为0
          limits_.cell_limits().num_x_cells * limits_.cell_limits().num_y_cells,
          kUnknownCorrespondenceValue),  // 0
      min_correspondence_cost_(min_correspondence_cost),  // 0.1
      max_correspondence_cost_(max_correspondence_cost),  // 0.9
      // 返回转换表，并初始化给成员变量
      value_to_correspondence_cost_table_(conversion_tables->GetConversionTable(
          max_correspondence_cost, min_correspondence_cost, max_correspondence_cost))
{
  CHECK_LT(min_correspondence_cost_, max_correspondence_cost_);
}

Grid2D::Grid2D(const proto::Grid2D& proto,
               ValueConversionTables* conversion_tables)
    : limits_(proto.limits()),
      correspondence_cost_cells_(),
      min_correspondence_cost_(MinCorrespondenceCostFromProto(proto)),
      max_correspondence_cost_(MaxCorrespondenceCostFromProto(proto)),
      value_to_correspondence_cost_table_(conversion_tables->GetConversionTable(
          max_correspondence_cost_, min_correspondence_cost_,
          max_correspondence_cost_)) 
{
  CHECK_LT(min_correspondence_cost_, max_correspondence_cost_);
  if (proto.has_known_cells_box()) 
  {
    const auto& box = proto.known_cells_box();
    known_cells_box_ =
        Eigen::AlignedBox2i(Eigen::Vector2i(box.min_x(), box.min_y()),
                            Eigen::Vector2i(box.max_x(), box.max_y()));
  }
  correspondence_cost_cells_.reserve(proto.cells_size());
  for (const auto& cell : proto.cells()) 
  {
    CHECK_LE(cell, std::numeric_limits<uint16>::max());
    correspondence_cost_cells_.push_back(cell);
  }
}

// Finishes the update sequence.
// 插入雷达数据结束
void Grid2D::FinishUpdate()
{
  // 记录已经更新过的索引，不为空
  while (!update_indices_.empty()) 
  {
    // 栅格值 >= kUpdateMarker，条件说明之前更新过了，更新过了才能结束更新
    DCHECK_GE(correspondence_cost_cells_[update_indices_.back()],
              kUpdateMarker);
    
    // 更新的时候加上了kUpdateMarker, 在这里减去
    correspondence_cost_cells_[update_indices_.back()] -= kUpdateMarker;
    // 把之前保存索引的删除，直到索引为空
    update_indices_.pop_back();
  }
}

// Fills in 'offset' and 'limits' to define a subregion of that contains all
// known cells.
// 根据known_cells_box_更新limits
void Grid2D::ComputeCroppedLimits(Eigen::Array2i* const offset,
                                  CellLimits* const limits) const 
{
  if (known_cells_box_.isEmpty())
  {
    *offset = Eigen::Array2i::Zero();
    *limits = CellLimits(1, 1);
    return;
  }
  // boundingbox的最小坐标，最接近地图左上角的值
  *offset = known_cells_box_.min().array();
  // boundingbox的size
  *limits = CellLimits(known_cells_box_.sizes().x() + 1,
                       known_cells_box_.sizes().y() + 1);
}

// Grows the map as necessary to include 'point'. This changes the meaning of
// these coordinates going forward. This method must be called immediately
// after 'FinishUpdate', before any calls to 'ApplyLookupTable'.

// 根据bounding_box的最小最大坐标，决定是否对地图进行扩大
void Grid2D::GrowLimits(const Eigen::Vector2f& point)
{
  GrowLimits(point,// bounding_box的最小or最大坐标
  {mutable_correspondence_cost_cells()},// 该容器储存地图栅格值, 存储的是free的概率转成uint16后的[0, 32767]范围内的值, 0代表未知
             {kUnknownCorrespondenceValue});// 容器(存放0)
}


/*
cartographer的地图物理坐标系
中心点不变, 子图向四周扩大两倍
new_max
o——————————————————
|    old_max   →  |
|    o————————    |
|    |   .   | → y_offset
|    |___↓___|    |                     ^ x
| ↑    ↑_x_offset |                     |
|_________________|                     |
                                        |
                                        |
                    y <—————————————————o
*/

// 根据物理坐标决定是否对地图进行扩大
void Grid2D::GrowLimits(const Eigen::Vector2f& point,
                        // grids容器存放的是容器指针
                        const std::vector<std::vector<uint16>*>& grids,
                        const std::vector<uint16>& grids_unknown_cell_values)
{
  CHECK(update_indices_.empty());

  // 计算buonding_box最大的物理坐标点的对应像素坐标，判断该像素坐标是否在地图坐标系内，
  // 如果像素坐标是 不在 栅格地图内部，则对地图进行扩大两倍，如果一直不在则加倍扩大
  while (!limits_.Contains(limits_.GetCellIndex(point)))
  {
    // 原栅格地图x、y方向上栅格的个数/2 = 50
    const int x_offset = limits_.cell_limits().num_x_cells / 2;
    const int y_offset = limits_.cell_limits().num_y_cells / 2;

    // 重新计算地图边界：将xy扩大至2倍, 中心点不变, 向四周扩大
    const MapLimits new_limits(
        limits_.resolution(),// 分辨率不变
        // 最大值扩大两倍
        limits_.max() + limits_.resolution() * Eigen::Vector2d(y_offset, x_offset),// 100 
        // x、y方向上的栅格数扩大两倍
        CellLimits(2 * limits_.cell_limits().num_x_cells,
                   2 * limits_.cell_limits().num_y_cells));
    
    // 新栅格地图x方向上栅格的个数
    const int stride = new_limits.cell_limits().num_x_cells;
    // 老坐标系的原点在新坐标系下的一维像素坐标
    const int offset = x_offset + stride * y_offset;

    // 新地图尺寸，或者是新地图所有栅格数量
    const int new_size = new_limits.cell_limits().num_x_cells *
                         new_limits.cell_limits().num_y_cells;

    // grids.size()开始为1
    for (size_t grid_index = 0; grid_index < grids.size(); ++grid_index) 
    {
      // 创建一个空的新地图，新地图所有栅格的栅格值初始化为0
      std::vector<uint16> new_cells(new_size, grids_unknown_cell_values[grid_index]);
  
      // 遍历老地图的所有栅格
      for (int i = 0; i < limits_.cell_limits().num_y_cells; ++i) 
      {
        for (int j = 0; j < limits_.cell_limits().num_x_cells; ++j) 
        {
          // 复制到将老地图的栅格值新地图上，但新地图内老地图外的栅格的值仍是0:未知
          // 老地图的第0个像素索引，对应新地图的第offset个
          new_cells[offset + j + i * stride] =
              (*grids[grid_index])[j + i * limits_.cell_limits().num_x_cells];
              // 取出第grid_index容器的内容(容器指针)，对指针进行解引用得到容器，根据索引取出容器中的内容
        }
      }

      // 将新地图替换覆盖老地图, 拷贝
      *grids[grid_index] = new_cells;
    }

    // 更新地图尺寸
    limits_ = new_limits;

    // 如果像素坐标不为空
    if (!known_cells_box_.isEmpty())
    {
      // 地图的尺寸变了
      // 将known_cells_box_的x与y进行平移到老地图的范围上??
      known_cells_box_.translate(Eigen::Vector2i(x_offset, y_offset));
    }

  }// end while
}

proto::Grid2D Grid2D::ToProto() const 
{
  proto::Grid2D result;
  *result.mutable_limits() = mapping::ToProto(limits_);
  *result.mutable_cells() = {correspondence_cost_cells_.begin(),
                             correspondence_cost_cells_.end()};
  CHECK(update_indices().empty()) << "Serializing a grid during an update is "
                                     "not supported. Finish the update first.";
  if (!known_cells_box().isEmpty()) {
    auto* const box = result.mutable_known_cells_box();
    box->set_max_x(known_cells_box().max().x());
    box->set_max_y(known_cells_box().max().y());
    box->set_min_x(known_cells_box().min().x());
    box->set_min_y(known_cells_box().min().y());
  }
  result.set_min_correspondence_cost(min_correspondence_cost_);
  result.set_max_correspondence_cost(max_correspondence_cost_);
  return result;
}

}  // namespace mapping
}  // namespace cartographer
