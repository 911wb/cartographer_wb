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

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

/**
 * note: gflags是一套命令行参数解析工具 
 * DEFINE_bool在gflags.h中定义
 * gflags主要支持的参数类型包括bool, int32, int64, uint64, double, string等
 * 定义参数通过DEFINE_type宏实现, 该宏的三个参数含义分别为命令行参数名, 参数默认值, 以及参数的帮助信息
 * 当参数被定义后, 通过FLAGS_name就可访问到对应的参数
 */

// collect_metrics ：激活运行时度量的集合.如果激活, 可以通过ROS服务访问度量
// 参数名，默认值，说明信息  ，当参数被定义后用：FLAGS_参数名 就可以访问到参数值
DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
// lua文件地址，从launch文件中传进来
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
// lua文件名称，从launch文件中传进来
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
// pbstream文件，从launch文件中传进来
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");

DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros {
namespace {

// cartographer_ros没有设计算法，而是处理传感器数据到cartographer，
// 并从cartographer中获取轨迹信息，地图信息，再进行ros话题的发布和可视化显示
void Run() 
{
  // 缓存的时间
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  // 调用构造函数初始化，建立缓冲区
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};

  // 创建订阅对象，将订阅的数据缓存到tf_buffer 一旦创建了监听器，它就开始通过线路接收tf转换
  // 开启监听tf的独立线程，子线程
  tf2_ros::TransformListener tf(tf_buffer);

  // 创建结构体对象:用来存放lua文件中的部分参数，后面赋值
  NodeOptions node_options;
  // 创建结构体对象:用来存放lua文件中的部分参数，后面赋值
  TrajectoryOptions trajectory_options;

  /*
  c++11: 
  std::tie会将变量node_options, trajectory_options 的引用整合成一个tuple，从而实现批量赋值
  LoadOptions()加载lua文件的内容并返回一个tuple，分别赋值给node_options, trajectory_options
  */
  
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  // notice: 实例化地图构建器，map_builder是指向MapBuilderInterface类的智能指针
  // MapBuilder类是完整的SLAM算法类包含前端(TrajectoryBuilders,scan to submap) 与 后端(用于查找回环的PoseGraph)
  auto map_builder =
      // 返回指向MapBuilder类的智能指针
      cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  

  /*
  c++11: std::move 是将对象的状态或者所有权从一个对象转移到另一个对象, 移动后map_builder指向空
  只是转移, 没有内存的搬迁或者内存拷贝所以可以提高利用效率,改善性能..
  右值引用是用来支持转移语义的.转移语义可以将资源 ( 堆, 系统对象等 ) 从一个对象转移到另一个对象, 
  这样能够减少不必要的临时对象的创建、拷贝以及销毁, 能够大幅度提高 C++ 应用程序的性能.
  临时对象的维护 ( 创建和销毁 ) 对性能有严重影响.
  */
  
  // Node类的初始化, 将ROS的topic传入SLAM, 也就是MapBuilder
  // 入参：lua文件参数、指向MapBuilder类的智能指针、缓存区接收数据的地址
  // map_builder是左值，std::move(map_builder)是右值
  Node node(node_options, std::move(map_builder), &tf_buffer,
            FLAGS_collect_metrics);

  // 如果加载了pbstream文件(地图文件，纯定位), 就执行LoadState
  if (!FLAGS_load_state_filename.empty())
  {
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }

  // 使用默认topic，开始轨迹，默认true
  if (FLAGS_start_trajectory_with_default_topics) 
  {
    
    // 入参:部分lua文件参数
    node.StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  // 与回调函数适配使用，单线程，按照顺序不断循环执行前面的node类回调函数，直到ctrl + C才往下走 
  ::ros::spin();

  // ctrl + C 到达这步： 结束所有处于活动状态的轨迹
  node.FinishAllTrajectories();

  // 当所有的轨迹结束时, 再执行一次全局优化
  // 轨迹结束后，不需要实时的计算，有更长的时间进行优化
  node.RunFinalOptimization();

  // 保存轨迹：如果save_state_filename非空, 就保存pbstream文件
  if (!FLAGS_save_state_filename.empty()) 
  {
    node.SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros

// step1  ?? notice: 公式： 改进： c++11:
// 启动一个新的节点cartographer_node
int main(int argc, char** argv) 
{

  // note: 初始化glog库  argv[0] = 可执行程序，默认就有
  google::InitGoogleLogging(argv[0]);
  
  // 使用gflags进行参数的初始化. 其中第三个参数为remove_flag
  // 如果为true, gflags会移除parse过的参数, 否则gflags就会保留这些参数, 但可能会对参数顺序进行调整.
  // 赋值给DEFINE_string中的参数，后面加上FLAGS_之后就可以使用
  google::ParseCommandLineFlags(&argc, &argv, true);

  /**
   * @brief glog里提供的CHECK系列的宏, 检测某个表达式是否为真
   * 检测expression如果不为真, 则打印后面的description和栈上的信息
   * 然后退出程序, 出错后的处理过程和FATAL比较像.
   */
  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  // ::是全局作用域，相当于根目录，ros节点的初始化，节点名称：cartographer_node
  ::ros::init(argc, argv, "cartographer_node");

  // 一般不需要在自己的代码中显式调用，但是若想在创建任何NodeHandle实例之前启动ROS相关的线程, 网络等, 可以显式调用该函数.
  ::ros::start();

  // 使用ROS_INFO进行glog消息的输出，调用构造函数初始化参数列表:AddLogSink(this)
  cartographer_ros::ScopedRosLogSink ros_log_sink;

  // 开始运行cartographer_ros
  cartographer_ros::Run();

  // 结束ROS相关的线程, 网络等
  ::ros::shutdown();
}
