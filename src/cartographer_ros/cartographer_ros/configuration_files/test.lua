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
  tracking_frame = "imu_link",
  --bag中，tf树最顶层的坐标系的下一层
  published_frame = "odom",
  odom_frame = "odom",
  --是否提供里程计，如果bag中有里程计的坐标系，这个就是false，如果没有就根据需要决定，如果改为true导致重复跳动
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  -- 使用位姿推测器，一定是false use_pose_extrapolator = false,
  
  -- 是否使用里程计的传感器数据，如果是true，tf树中一定存在odom坐标系，订阅里程计的topic
  use_odometry = false,
  -- 是否使用GPS数据
  use_nav_sat = false,
  use_landmarks = false,
  -- 单线点云的话题数量
  num_laser_scans = 0,
  -- 多回声波雷达
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  -- 多线点云的话题数量
  num_point_clouds = 1,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
-- 是否使用imu
TRAJECTORY_BUILDER_2D.use_imu_data = true
-- 多线点云最小的z范围，单线点云的z是0，不能设置以下参数，min_z以下的点云都不要了，min_z越大，打到地面上的点越少，点云位置越高
TRAJECTORY_BUILDER_2D.min_z = 0.2

-- TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10 重影

return options
