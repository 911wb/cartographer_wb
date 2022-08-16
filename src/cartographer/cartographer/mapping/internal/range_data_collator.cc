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

#include "cartographer/mapping/internal/range_data_collator.h"

#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/mapping/internal/local_slam_result_data.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

constexpr float RangeDataCollator::kDefaultIntensityValue;

/**
 * @brief 多个雷达数据的时间同步
 * 
 * @param[in] sensor_id 雷达数据的话题
 * @param[in] timed_point_cloud_data 雷达数据,时间同步前的点云
 * @return sensor::TimedPointCloudOriginData 根据时间处理之后的数据
 */
sensor::TimedPointCloudOriginData RangeDataCollator::AddRangeData(
    const std::string& sensor_id,
    // 时间同步前的点云，值传递，值=数据别名
    sensor::TimedPointCloudData timed_point_cloud_data)
{
  // 检查话题是否传错
  CHECK_NE(expected_sensor_ids_.count(sensor_id), 0);

  // 从sensor_bridge传过来的数据的intensities为空
  timed_point_cloud_data.intensities.resize(
      timed_point_cloud_data.ranges.size(), kDefaultIntensityValue);

  // TODO(gaschler): These two cases can probably be one.
  // 如果同话题的点云, 还有没处理的, 就先处理的, 将当前点云保存
  // 一开始为0，因为没有往容器中添加，scan_1和scan_2都不进入
  if (id_to_pending_data_.count(sensor_id) != 0)
  {
    // current_end_：上一次时间同步的结束时间
    // current_start_：本次时间同步的开始时间
    // 上一次的结束作为本次的开始，保证时间连续
    current_start_ = current_end_;
    // When we have two messages of the same sensor, move forward the older of
    // the two (do not send out current).
    // 本次时间同步的结束时间 设置为当前帧点云数据(本次待处理，但未处理)的结束时间(最后一个点的时间)
    current_end_ = id_to_pending_data_.at(sensor_id).time;
    // 数据的裁剪与合并
    auto result = CropAndMerge();
    // 将当前的点云数据储存到，待处理的的点云容器中
    id_to_pending_data_.emplace(sensor_id, std::move(timed_point_cloud_data));

    // 返回从start到end这段时间内处理的结果
    return result;
  }

  // 将当前点云数据添加到等待时间同步的容器中，添加scan_1和scan_2的数据
  id_to_pending_data_.emplace(sensor_id, std::move(timed_point_cloud_data));

  // 等到range数据的所有话题(scan_1、scan_2...)，到来之后再进行处理
  if (expected_sensor_ids_.size() != id_to_pending_data_.size()) 
  {
    // 等待所有话题，到齐后再进行时间同步
    // 返回空的TimedPointCloudOriginData结构体对象
    return {};
  }

  // 上一次结束时间为本次的开始时间
  current_start_ = current_end_;
  // We have messages from all sensors, move forward to oldest.
  common::Time oldest_timestamp = common::Time::max();

  // 找到所有传感器数据中最早(晚)的时间戳(点云最后一个点的绝对时间，相对时间为0)
  // 从scan_1和scan_2中找到最晚的时间戳 = timestamp2
  for (const auto& pair : id_to_pending_data_)
  {
    // 最早(晚)的时间戳
    oldest_timestamp = std::min(oldest_timestamp, pair.second.time);
  }
  
  // 最后当前一个点的绝对时间(一帧激光的总时长)，作为本次时间同步的结束时间
  // 是待时间同步map中的，所有点云中最早的时间戳
  current_end_ = oldest_timestamp;// = timestamp2
  // 数据的裁剪与合并
  return CropAndMerge();
}
// notice:

// 对时间段内的数据进行截取与合并, 返回时间同步后的点云
// 情况1裁剪：如果只有一个雷达传感器传入数据：当前这一帧与前一帧数据在时间上交错时，就把当前帧相交的那一部分忽略，储存相交的部分
// 情况2合并：如果有两个或多个雷达传感器传入数据，我们把这些传感器的一帧的数据都收集齐，把它们合并在一起，按照绝对时间排放
sensor::TimedPointCloudOriginData RangeDataCollator::CropAndMerge() 
{
  // 初始化结构体，存放结果
  sensor::TimedPointCloudOriginData result{current_end_, {}, {}};
  bool warned_for_dropped_points = false;

  // 遍历所有的测距传感器话题 std::map<std::string, sensor::TimedPointCloudData> id_to_pending_data_; 
  // 遍历scan_1和scan_2的数据
  for (auto it = id_to_pending_data_.begin(); it != id_to_pending_data_.end();) 
  {
    // 当前传感器的所有数据
    sensor::TimedPointCloudData& data = it->second;
    // 当前传感器的测距值的引用
    const sensor::TimedPointCloud& ranges = it->second.ranges;
    // 空
    const std::vector<float>& intensities = it->second.intensities;

    // no1:第一个测距值，scan_1的第一个数据
    // no2:scan_2的第一个数据
    auto overlap_begin = ranges.begin();
    
    // 第一次不进来：第一个点的绝对时间>current_start_ = min
    // no2:
    while (overlap_begin < ranges.end() &&
           data.time + common::FromSeconds((*overlap_begin).time) < current_start_) 
    {
      // 当前传感器点云最后一个点的绝对时间 + 当前点的相对时间(负的)= 每一个点的绝对时间戳 < current_start_(上一帧结束的时间作为当前帧的开始时间)

      // 指针指向不断下一个测距值，直到
      ++overlap_begin;
    }

    // 找到点云中 最后一个时间戳小于等于current_end_的点的索引
    auto overlap_end = overlap_begin;

    // no1：第一个点的绝对时间 < current_end_ = timestamp2
    // no2：第一个点的绝对时间 < current_end_ = timestamp2
    while (overlap_end < ranges.end() &&
           data.time + common::FromSeconds((*overlap_end).time) <= current_end_) 
    {
      // 当前传感器点云最后一个点的绝对时间 + 当前点的相对时间(负的)= 每一个点的绝对时间戳 <= current_end_(当前帧最后一个点的绝对时间)
      // no1:直达遍历完scan_1的最后一个数据，退出循环
      // no2:直达遍历完scan_2的最后一个数据，此时overlap_end =timestamp2， 退出循环
      ++overlap_end;
    }

    // 丢弃点云中时间比起始时间早的点, 每执行一下CropAndMerge()打印一次log
    // no1、2：不进来因为ranges.begin() = overlap_begin，数据交错时进来
    if (ranges.begin() < overlap_begin && !warned_for_dropped_points) 
    {
      // 告诉用户由数据交错
      LOG(WARNING) << "Dropped " << std::distance(ranges.begin(), overlap_begin)
                   << " earlier points.";
      // 不使用交错数据
      warned_for_dropped_points = true;
    }

    // Copy overlapping range.
    // no1:第一次循环时进来，no2:第二次循环时进来
    if (overlap_begin < overlap_end) 
    {
      // 获取下个点云的index, 即当前vector的个数，一开始是0
      std::size_t origin_index = result.origins.size();
      // 插入当前点云的原点坐标
      result.origins.push_back(data.origin);

      // 获取此传感器时间与集合时间戳的误差，时间修正
      const float time_correction =
          static_cast<float>(common::ToSeconds(data.time - current_end_));

      auto intensities_overlap_it =
          intensities.begin() + (overlap_begin - ranges.begin());
      
      // reserve() 预留空间改变
      result.ranges.reserve(result.ranges.size() +
                            std::distance(overlap_begin, overlap_end));
      
      // no1:遍历scan_1中overlap_begin到overlap_end之间的数据
      for (auto overlap_it = overlap_begin; overlap_it != overlap_end;
           ++overlap_it, ++intensities_overlap_it) 
      {
        sensor::TimedPointCloudOriginData::RangeMeasurement point{
            *overlap_it, *intensities_overlap_it, origin_index};
        // current_end_ + point_time[3]_after == in_timestamp +
        // point_time[3]_before

        // 针对每个点时间戳进行修正, 让最后一个点的时间为0
        point.point_time.time += time_correction;

        // 只将overlap_begin到overlap_end之间的数据，添加给result
        // no1:储存scan_1的所有数据，no2:储存scan_2的所有数据 --> 数据合并
        result.ranges.push_back(point);
      } // end for
    } // end if

    // no1:overlap_end指针移动到最后一个测距值
    // no2:overlap_end指针移动到最后一个测距值
    if (overlap_end == ranges.end()) 
    {
      // 点云每个点都用了,则可将这一帧数据进行删除
      // 删除scan_1，进入下一个循环
      it = id_to_pending_data_.erase(it);
    }
    // 如果一个点都没用, 就先放这, 看下一个数据
    // 指针还在第一个地方
    else if (overlap_end == ranges.begin()) 
    {
      ++it;
    } 
    // 指针不在第一个位置也不在最后一个位置：用了一部分的点
    else 
    {
      const auto intensities_overlap_end =
          intensities.begin() + (overlap_end - ranges.begin());
      // 将用了的点删除, 这里的赋值是拷贝
      data = sensor::TimedPointCloudData{
          data.time, data.origin,
          sensor::TimedPointCloud(overlap_end, ranges.end()),
          std::vector<float>(intensities_overlap_end, intensities.end())};
      ++it;
    }
  } // end for

  // 对各传感器的点云 按照每个点的时间从小到大进行排序
std::sort(result.ranges.begin(), result.ranges.end(),
          [](const sensor::TimedPointCloudOriginData::RangeMeasurement& a,
              const sensor::TimedPointCloudOriginData::RangeMeasurement& b) 
          {
            // time：是相对于点云最后一个点(时间为0)的相对时间，是负的
            return a.point_time.time < b.point_time.time;
          });

  return result;
}

}  // namespace mapping
}  // namespace cartographer
