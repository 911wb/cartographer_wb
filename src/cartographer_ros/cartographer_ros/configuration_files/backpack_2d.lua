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
-- options中的内容被赋值给，TrajectoryOptions、NodeOption结构体创建的对象
options = {
  map_builder = MAP_BUILDER,                -- map_builder.lua的配置信息
  trajectory_builder = TRAJECTORY_BUILDER,  -- trajectory_builder.lua的配置信息
  
  map_frame = "map",                        -- 地图坐标系的名字
  -- imu的发布数据的频率是400，雷达的20，将雷达的数据转换到imu坐标系，计算量很小
  tracking_frame = "base_footprint",              -- 将所有传感器数据通过坐标变换转换到这个坐标系下
  published_frame = "odom",            -- tf: map -> odom ， 修改前：footprint
  odom_frame = "odom",                      -- 里程计的坐标系名字
  provide_odom_frame = false,               -- 是否提供odom的tf, 如果为true,则tf树为map->odom->footprint(odom) 重名
                                            -- 如果为false tf树为map->footprint
  publish_frame_projected_to_2d = false,    -- 是否将坐标系投影到平面上
-- use_pose_extrapolator = true,            -- 发布tf时是使用pose_extrapolator的位姿还是前端计算出来的位姿

  use_odometry = true,                     -- 是否使用里程计,如果使用要求一定要有odom的tf

  use_nav_sat = false,                      -- 是否使用gps
  use_landmarks = false,                    -- 是否使用landmark
  num_laser_scans = 1,                      -- 是否使用单线激光数据
  num_multi_echo_laser_scans = 0,           -- 是否使用multi_echo_laser_scans数据
  num_subdivisions_per_laser_scan = 1,      -- 1帧数据被分成几次处理,一般为1
  num_point_clouds = 0,                     -- 是否使用点云数据 ，是否使用多线激光雷达
  
  lookup_transform_timeout_sec = 0.2,       -- 查找tf时的超时时间
  submap_publish_period_sec = 0.3,          -- 发布submap的时间间隔，0.3秒1次
  pose_publish_period_sec = 5e-3,           -- 发布位姿的频率
  trajectory_publish_period_sec = 30e-3,    -- 发布轨迹的频率

  rangefinder_sampling_ratio = 1.,          -- 传感器数据的采样频率：来n帧用一帧
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.use_imu_data = true

--TRAJECTORY_BUILDER_2D.min_range = 0.3
--TRAJECTORY_BUILDER_2D.max_range = 100.
--TRAJECTORY_BUILDER_2D.min_z = 0.5 -- 点云的高度小于min_z就会舍弃，单线点云不能设置(单线点云的z是0)，多线点云的min_z要大于0，由于多线点云数据会打到地面，如果要建立二维地图min_z=0，地图上就会出现印子，导致地图无法使用，所有min_z要大于0



--TRAJECTORY_BUILDER_2D.max_z = 1.4
--TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.02

--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200.
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 50.

TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.9
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 100
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 50.

--TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- 建图效果调整
-- 点云和地图匹配的程度，位姿残差×权重，权重越大，越相信匹配结果
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
-- 帧间匹配位姿与先验位姿的残差×权重，权重越大，越相信先验位姿的结果
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.
-- imu不好时，可以调小该权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.


--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 12

TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.004
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 1.

-- 一个子图插入160个节点，这个数越小，生成的子图越多，构造多分辨率地图时，总的子图变得更多，占用内存大
--TRAJECTORY_BUILDER_2D.submaps.num_range_data = 80.
-- 地图分辨率
--TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.1

-- 每插入n个节点执行优化一次
POSE_GRAPH.optimize_every_n_nodes = 160.
-- 约束的采样频率
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
-- 回环检测，最大距离
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.

-- 回环检测错误匹配时，调整的参数
-- 回环检测阈值
POSE_GRAPH.constraint_builder.min_score = 0.48
-- 回环检测全局地图匹配的阈值
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60

return options
