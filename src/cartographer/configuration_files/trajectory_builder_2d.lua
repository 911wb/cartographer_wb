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

TRAJECTORY_BUILDER_2D = {
  use_imu_data = true,            -- 是否使用imu数据
  min_range = 0.,                 -- 雷达数据的最远最近滤波, 保存中间值
  max_range = 30.,                -- 雷达数据的使用范围 单位：m，如果该值太大，需要处理的雷达点数增多计算量增大、远距离的点不准
  min_z = -0.8,                   -- 雷达数据的最高与最低的过滤, 保存中间值
  max_z = 2.,                     -- 超过2m就不用
  missing_data_ray_length = 5.,   -- 超过最大距离范围的数据点用这个距离代替，太远了不返回
  num_accumulated_range_data = 1, -- 几帧有效的点云数据进行一次扫描匹配，如果是1，那就是一帧一帧的处理雷达 数据
  voxel_filter_size = 0.025,      -- 体素滤波的立方体的边长2.5cm，增大边长相当于增大增大正方体，一个点云代表代表更多点云，减少了点云，也就减少了计算量

  -- 使用固定的voxel滤波之后, 再使用自适应体素滤波器
  -- 自适应体素滤波器 用于生成稀疏点云 以进行 扫描匹配
  adaptive_voxel_filter = {
    max_length = 0.5,             -- 尝试确定最佳的立方体边长, 边长最大为0.5
    min_num_points = 200,         -- 如果存在 较多点 并且大于'min_num_points', 则减小体素长度以尝试获得该最小点数
    max_range = 50.,              -- 距远离原点超过max_range 的点被移除
  },

  -- 闭环检测的自适应体素滤波器, 用于生成稀疏点云 以进行 闭环检测
  loop_closure_adaptive_voxel_filter = {
    max_length = 0.9,
    min_num_points = 100,
    max_range = 50.,
  },

  -- 在扫描匹配之前，是否要使用一种计算量很高的方式来计算位姿，只使用单线雷达数据时，使用
  -- 是否使用暴力匹配为ceres提供先验信息，没有IMU或里程计时使用
  -- 计算复杂度高 , 但是很鲁棒 , 在odom或者imu不准时依然能达到很好的效果
  use_online_correlative_scan_matching = false,
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.1,             -- 线性搜索窗口的大小
    angular_search_window = math.rad(20.),  -- 角度搜索窗口的大小
    translation_delta_cost_weight = 1e-1,   -- 用于计算各部分score的权重
    rotation_delta_cost_weight = 1e-1,
  },

  -- ceres匹配的一些配置参数
  ceres_scan_matcher = {
    occupied_space_weight = 1., -- 点云与地图匹配的权重
    translation_weight = 10., -- 先验位姿和帧间匹配位姿(后验)的残差的权重
    rotation_weight = 40., -- 先验位姿和帧间匹配位姿(后验)的残差的权重
    ceres_solver_options = 
    {
      use_nonmonotonic_steps = false,
      max_num_iterations = 20, -- 最大迭代次数
      num_threads = 1,-- 使用的线程数
    },
  },

  -- 为了防止子图里插入太多数据, 在插入子图之前之前对数据进行过滤
  motion_filter = {
    max_time_seconds = 5.,
    max_distance_meters = 0.2,
    max_angle_radians = math.rad(1.),
  },

  -- TODO(schwoere,wohe): Remove this constant. This is only kept for ROS.
  imu_gravity_time_constant = 10.,

  -- 位姿预测器相关
  pose_extrapolator = {
    use_imu_based = false,
    constant_velocity = {
      imu_gravity_time_constant = 10.,
      pose_queue_duration = 0.001,
    },
    imu_based = {  -- 没有实现imubase
      pose_queue_duration = 5.,
      gravity_constant = 9.806,
      pose_translation_weight = 1.,
      pose_rotation_weight = 1.,
      imu_acceleration_weight = 1.,
      imu_rotation_weight = 1.,
      odometry_translation_weight = 1.,
      odometry_rotation_weight = 1.,
      solver_options = {
        use_nonmonotonic_steps = false;
        max_num_iterations = 10;
        num_threads = 1;
      },
    },
  },

  -- 子图相关的一些配置
  submaps = {
    num_range_data = 90,          -- 90 建立一张子图需要的点云帧数为num_range_data*2
    grid_options_2d = 
    {
      grid_type = "PROBABILITY_GRID", -- 概率地图，地图的种类, 还可以是tsdf格式
      resolution = 0.05, -- 地图分辨率 室外：0.1，室内0.05，值越小精度越高，占用内存越大，分辨率越粗，匹配时的搜索量越小
    },
    range_data_inserter = {
      -- switch
      range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
      -- 概率占用栅格地图的一些配置
      probability_grid_range_data_inserter = {
        insert_free_space = true,
        hit_probability = 0.55,
        miss_probability = 0.49,
      },
      -- tsdf地图的一些配置
      tsdf_range_data_inserter = {
        truncation_distance = 0.3,
        maximum_weight = 10.,
        update_free_space = false,
        normal_estimation_options = {
          num_normal_samples = 4,
          sample_radius = 0.5,
        },
        project_sdf_distance_to_scan_normal = true,
        update_weight_range_exponent = 0,
        update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0.5,
        update_weight_distance_cell_to_hit_kernel_bandwidth = 0.5,
      },
    },
  },
}
