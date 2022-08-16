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

#include "cartographer/sensor/internal/ordered_multi_queue.h"

#include <algorithm>
#include <sstream>
#include <vector>

#include "absl/memory/memory.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {

namespace {

// Number of items that can be queued up before we log which queues are waiting
// for data.
const int kMaxQueueSize = 500;

}  // namespace

inline std::ostream& operator<<(std::ostream& out, const QueueKey& key) 
{
  return out << '(' << key.trajectory_id << ", " << key.sensor_id << ')';
}

OrderedMultiQueue::OrderedMultiQueue() {}

OrderedMultiQueue::~OrderedMultiQueue() 
{
  for (auto& entry : queues_) 
  {
    CHECK(entry.second.finished);
  }
}

/**
 * @brief 添加一个数据队列,并保存回调函数 CollatedTrajectoryBuilder::HandleCollatedSensorData
 * 
 * @param[in] queue_key 轨迹id与topic名字
 * @param[in] callback void(std::unique_ptr<Data> data) 型的函数
 * 这里的callback已经是对应sensor_id的callback了
 */
void OrderedMultiQueue::AddQueue(const QueueKey& queue_key, Callback callback) 
{
  CHECK_EQ(queues_.count(queue_key), 0);
  // 有多少个传感器数据队列就赋值了多少个回调函数
  // using Callback = std::function<void(std::unique_ptr<Data>)>;
  queues_[queue_key].callback = std::move(callback);
}

// 将queue_key对应的Queue的finished设置成true
void OrderedMultiQueue::MarkQueueAsFinished(const QueueKey& queue_key) 
{
  auto it = queues_.find(queue_key);
  CHECK(it != queues_.end()) << "Did not find '" << queue_key << "'.";

  auto& queue = it->second;
  CHECK(!queue.finished);

  queue.finished = true;
  // notice: 传感器数据的分发处理，数据同步
  Dispatch();
}

// 向数据队列中添加数据
void OrderedMultiQueue::Add(const QueueKey& queue_key,
                            std::unique_ptr<Data> data) 
{
  auto it = queues_.find(queue_key);
  // 如果queue_key不在queues_中, 就忽略data
  if (it == queues_.end()) 
  {
    LOG_EVERY_N(WARNING, 1000)
        << "Ignored data for queue: '" << queue_key << "'";
    return;
  }

  // 向数据队列中添加数据
  it->second.queue.Push(std::move(data));

  // notice: 传感器数据的分发处理，数据同步 将处于数据队列中的数据根据时间依次传入回调函数(数据分发)
  Dispatch();
}

// 将所有处于未完成状态的数据队列标记为完成状态
void OrderedMultiQueue::Flush() 
{
  // 找到所有unfinished的数据队列
  std::vector<QueueKey> unfinished_queues;
  for (auto& entry : queues_) {
    if (!entry.second.finished) 
    {
      unfinished_queues.push_back(entry.first);
    }
  }
  // 将unfinished_queues标记为完成状态
  for (auto& unfinished_queue : unfinished_queues) 
  {
    MarkQueueAsFinished(unfinished_queue);
  }
}

// 返回阻塞的队列的QueueKey
QueueKey OrderedMultiQueue::GetBlocker() const 
{
  CHECK(!queues_.empty());
  return blocker_;
}

/**
 * @brief 将处于数据队列中的数据根据时间依次传入回调函数(数据分发)
 * 
 * 3种退出情况:
 * 退出条件1 某个话题的数据队列为空同时又不是完成状态, 就退出
 * 退出条件2 只有多队列queues_为空, 就退出
 * 退出条件3 数据队列中数据的个数只有1个,又不是完成状态,不能确定状态, 就先退出
 */
void OrderedMultiQueue::Dispatch() 
{
  while (true) 
  {
    /*
      // 多个传感器，每个传感器有一个队列
      std::map<QueueKey, Queue> queues_;
      struct QueueKey 
      {
        int trajectory_id;      // 轨迹id
        std::string sensor_id;  // topic名字
      }
      struct Queue 
      {
        // 指向传感器队列数据的对象 ，模板类泛型是智能指针
        common::BlockingQueue<std::unique_ptr<Data>> queue;   // 存储数据的队列
        Callback callback; // 带一个参数                        // 本数据队列对应的回调函数
        bool finished = false;                                // 这个queue是否finished
      };
      <键：值>
      (0, scan): {      4,     }
      (0, imu):  {1,  3,   5,  }
      (0, odom): {  2,       6,}
      --------------------------------->time
      上面最早的数据是imu的第一个数据
    */

    // 即将处理的数据
    const Data* next_data = nullptr;
    Queue* next_queue = nullptr;
    QueueKey next_queue_key;

    // step1 遍历所有的数据队列, 找到所有数据队列的第一个数据中时间最早(最开始)的一个数据
    // std::map<QueueKey, Queue> queues_; 多个传感器数据队列
    for (auto it = queues_.begin(); it != queues_.end();) 
    {
      // c++11: auto*(指针类型说明符), auto&(引用类型说明符), auto &&(右值引用)

      // 获取当前一个传感器队列中时间最早的的一个数据，it->second = 结构体对象Queue，Data是泛型代表任意传感器数据
      const auto* data = it->second.queue.Peek<Data>();
      // 如果当前队列为空
      if (data == nullptr)
      {
        // 如果队列已经处于finished状态了
        if (it->second.finished) 
        {
          // 删除该传感器队列
          queues_.erase(it++);
          continue;
        }
        // 退出条件1: 某个话题的数据队列为空(话题设置错误数据没到、消费者使用数据太快数据用完了)
        // 同时又不是完成状态, 就先退出, 发布log并标记为阻塞者
        CannotMakeProgress(it->first);// QueueKey
        return;
      }

      // 第一次进行到这里next_data == nullptr
      // data的时间(当前)比next_data(上一次)的时间小(早数据)，时间是递增的
      // 就更新next_data, 并保存当前话题的数据队列以及queue_key
      if (next_data == nullptr || data->GetTime() < next_data->GetTime()) 
      {
        // 上一次保存的时间，更新最早时间对应的数据，当前要处理的数据作为即将要处理的数据
        next_data = data;// 第一个数据(时间可能最早)
        //  it->second==Queue结构体，即将要处理的队列是其中一个传感器的队列，如储存一帧点云数据的队列
        next_queue = &it->second;
        // 队列名称
        next_queue_key = it->first;
      }

      // 数据的时间戳不是按顺序的, 就报错
      // 上一次最早时间 >= 当前最早时间
      CHECK_LE(last_dispatched_time_, next_data->GetTime())
          << "Non-sorted data added to queue: '" << it->first << "'";
      
      ++it;
    } // end for

    // 退出条件2: 只有多队列queues_为空, 才可能next_data==nullptr
    if (next_data == nullptr)
    {
      CHECK(queues_.empty());
      // 执行到这里说明一个传感器数据都没传入
      return;
    }

    // If we haven't dispatched any data for this trajectory yet, fast forward
    // all queues of this trajectory until a common start time has been reached.
    // 如果我们还没有为这个轨迹分配任何数据, 快进这个轨迹的所有队列, 直到达到一个共同的开始时间
    
    // step2 获取对应轨迹id的所有数据队列中的最早共同时间戳, 作为轨迹开始的时间
    // 每个传感器的数据队列不为空：遍历所有的队列，找到最早有数据(三个传感器数据都有)的时间戳, 作为轨迹开始的时间
    const common::Time common_start_time =
        GetCommonStartTime(next_queue_key.trajectory_id);

    // step3 将 next_queue 的时间最老(最早)的一个数据传入回调函数进行处理 

    // 大多数情况, 数据时间都会超过common_start_time的
    // 该传感器数据队列中的时间最早的数据，其对应的时间大于等于最小共同时间戳
    if (next_data->GetTime() >= common_start_time)
    {
      // 更新分发数据的时间
      last_dispatched_time_ = next_data->GetTime();

      // notice: 将某一个传感器的数据队列(如雷达传感器的一帧点云)传入 callback() 函数进行处理
      // next_queue是某个传感器数据的储存队列，每个传感器都有各自的回调函数处理各自的数据，如将一帧点云数据传入函数进行处理
      next_queue->callback(next_queue->queue.Pop());
    }
    // 数据时间小于common_start_time,同时数据队列中数据的个数小于2,只有1个数据的情况 罕见
    else if (next_queue->queue.Size() < 2) 
    {
      // 退出条件3: 数据队列数据的个数少,又不是完成状态, 不能确定现在到底是啥情况, 就先退出稍后再处理
      if (!next_queue->finished) 
      {
        // We cannot decide whether to drop or dispatch this yet.
        CannotMakeProgress(next_queue_key);
        return;
      } 
      // 处于完成状态了, 将数据传入 callback() 函数进行最后几个数据的处理
      // 更新分发数据的时间,将数据传入 callback() 进行处理,并将这个数据从数据队列中删除
      last_dispatched_time_ = next_data->GetTime();
      next_queue->callback(next_queue->queue.Pop());
    }
    // 数据时间小于common_start_time,同时数据队列数据的个数大于等于2个
    else 
    {
      // We take a peek at the time after next data. If it also is not beyond
      // 'common_start_time' we drop 'next_data', otherwise we just found the
      // first packet to dispatch from this queue.

      // 只处理数据在common_start_time的前一个数据, 其他更早的数据会被丢弃掉
      // 删除该队列的第一个数据，并赋值
      std::unique_ptr<Data> next_data_owner = next_queue->queue.Pop();
      if (next_queue->queue.Peek<Data>()->GetTime() > common_start_time) 
      {
        // 更新分发数据的时间,将数据传入 callback() 进行处理
        last_dispatched_time_ = next_data->GetTime();
        // 传入队列被删的除第一个数据的地址，即队列首地址给回调函数
        next_queue->callback(std::move(next_data_owner));
      }
    }
  }
}

// 标记queue_key为阻塞者,并按条件发布log,等等这个数据
void OrderedMultiQueue::CannotMakeProgress(const QueueKey& queue_key) 
{
  // 标记queue_key为阻塞者，当前队列为空的QueueKey
  blocker_ = queue_key;
  // 遍历所有传感器队列
  for (auto& entry : queues_) 
  {
    // queue_key对应的数据队列为空,而某一个传感器数据队列的数据已经大于500个了
    if (entry.second.queue.Size() > kMaxQueueSize) 
    {
      // 在该语句第1、61、121……次被执行的时候, 记录日志信息
      LOG_EVERY_N(WARNING, 60) << "Queue waiting for data: " << queue_key;// 轨迹id、话题名称
      // 话题名称设置不对时
      // [ WARN] [1628516438.493835120, 1606808659.273453929]: W0809 21:40:38.000000 10662 ordered_multi_queue.cc:230] Queue waiting for data: (0, points2)
      // [ WARN] [1628516439.089736487, 1606808659.869309184]: W0809 21:40:39.000000 10662 ordered_multi_queue.cc:230] Queue waiting for data: (0, points2)
      return;
    }
  }
}

/**
 * @brief 找到数据队列所有第一帧的最大时间(共同时间)
 * 对于某个id的轨迹的 common_start_time 只会计算一次
 * 
 * @param[in] trajectory_id 轨迹id
 * @return common::Time 返回数据队列所有第一帧的最大时间
 */
common::Time OrderedMultiQueue::GetCommonStartTime(const int trajectory_id) 
{

  // c++11: map::emplace() 返回的 pair 对象：(键，值)
  // pair 的成员变量 first 是一个指向插入元素或阻止插入的元素的迭代器
  // 成员变量 second 是个布尔值, 表示是否插入成功, 如果这个元素的索引已经存在插入会失败,返回false
  auto emplace_result = common_start_time_per_trajectory_.emplace(
      trajectory_id, common::Time::min());
  
  // 时间戳，初始值为最小值
  common::Time& common_start_time = emplace_result.first->second;

  // 如果插入成功了就找到时间戳最大的对common_start_time进行更新, 失败了就不更新
  // 只会在轨迹开始时插入成功一次
  if (emplace_result.second)
  {
    // 找到这个轨迹下,所有数据队列中数据的时间戳最大 的时间戳
    // 执行到这里时, 所有的数据队列都有值了, 因为没值的情况在Dispatch()中提前返回了
    // 遍历所有传感器队列
    for (auto& entry : queues_)
    {
      if (entry.first.trajectory_id == trajectory_id) 
      {
        // 遍历所有的队列，找到最早有数据(三个传感器数据都有)的时间戳，设置为common_start_time
        common_start_time = std::max(
            common_start_time, entry.second.queue.Peek<Data>()->GetTime());
      }
    }
    LOG(INFO) << "All sensor data for trajectory " << trajectory_id
              << " is available starting at '" << common_start_time << "'.";

    // [ INFO] [1628516134.243770381, 1606808649.533687125]: I0809 21:35:34.000000  8604 ordered_multi_queue.cc:264] All sensor data for trajectory 0 is available starting at '637424054495384530'.

  }

  return common_start_time;
}

}  // namespace sensor
}  // namespace cartographer