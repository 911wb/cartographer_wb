-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.
 
include "map_builder.lua"
include "trajectory_builder.lua"
 
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",

  --有imu的link就设置为imu_link，没有就设置为base_link
  tracking_frame = "base_link",
  --bag中，tf树最顶层的坐标系的下一层
  published_frame = "odom",

 --bag中，tf树最顶层的坐标系的下一层
  odom_frame = "odom",
--是否提供里程计，如果bag中有里程计的坐标系，这个就是false，如果没有就根据需要决定，如果改为true导致重复跳动
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,

-- 如果为true，则会订阅“odom”这个topic的的nav_msgs/Odometry消息
-- 是否使用里程计的传感器数据，如果是true，tf树中一定存在odom坐标系，订阅里程计的topic
  use_odometry = true,
-- 是否使用gps数据
  use_nav_sat = false,
-- 是否使用lankmarks数据
  use_landmarks = false,
-- 如果使用的是单线的激光雷达，此处为激光雷达的数量
  num_laser_scans = 1.,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1.,
-- 多线激光雷达的数量
  num_point_clouds = 0,

-- 重要参数 等待坐标变换发布信息的超时时间 0.3
  lookup_transform_timeout_sec = 0.3,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}
 
MAP_BUILDER.use_trajectory_builder_2d = true
 
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 8.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
 
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65
 
return options
