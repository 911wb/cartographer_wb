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

-- 这里的POSE_GRAPH指的就是位姿图，这里的参数全部与全局优化有关
POSE_GRAPH = {
  -- 每隔多少个节点执行一次后端全局优化 等于0时不进行优化, 这样就可以单独分析前端的效果，一般是num_range_data的两倍，该值越大，优化频率越低，计算量越小
  optimize_every_n_nodes = 90,

  -- 约束构建的相关参数，回环检测
  constraint_builder = {
    sampling_ratio = 0.3,                 --0.3 对局部子图进行回环检测时的计算频率, 数值越大, 计算次数越多约束越多，全局约束nodes采样比率，间隔1/0.3个node进行一次全局约束检测
    max_constraint_distance = 15.,        --15 对局部子图进行回环检测时能成为约束的最大距离 当前node与当前submap之间的距离小于15才建立全局约束
    min_score = 0.55,                     -- 对局部子图进行回环检测时的最低分数阈值，小于该阈值则回环检测失败 使用fast_CSM检测当前node与当前submap的匹配程度，当匹配得分超过0.55，才建立全局约束，阈值越大，约束越少
    global_localization_min_score = 0.6,  --0.6 对整体子图进行回环检测时的最低分数阈值，使用fast_CSM检测当前node与全局submap的匹配程度，当匹配得分超过0.6，才建立全局约束
    loop_closure_translation_weight = 1.1e4, --闭环检测平移权重
    loop_closure_rotation_weight = 1e5, --闭环检测旋转权重
    log_matches = true,                   -- 打印约束计算的log到终端 是否打印匹配结果
    
    -- 基于分支定界算法的2d粗匹配器
    fast_correlative_scan_matcher = {
      linear_search_window = 7., -- 线性搜索窗 
      angular_search_window = math.rad(30.), --匹配搜索角度
      branch_and_bound_depth = 7, -- 树的深度 分枝定界树的深度越小，生成的地图个数越少，占用内存越少，但是匹配的时间会增加
    },

    -- 基于ceres的2d精匹配器
    ceres_scan_matcher = {
      occupied_space_weight = 20., -- 匹配的占据空间权重
      translation_weight = 10., -- 匹配平移权重
      rotation_weight = 1., --匹配旋转权重
      ceres_solver_options = {
        use_nonmonotonic_steps = true, -- 梯度下降策略，防止陷入局部最小值
        max_num_iterations = 10, --最大迭代次数
        num_threads = 1, --使用几个线程
      },
    },

    -- 基于分支定界算法的3d粗匹配器
    fast_correlative_scan_matcher_3d = {
      branch_and_bound_depth = 8,
      full_resolution_depth = 3,
      min_rotational_score = 0.77,
      min_low_resolution_score = 0.55,
      linear_xy_search_window = 5.,
      linear_z_search_window = 1.,
      angular_search_window = math.rad(15.),
    },

    -- 基于ceres的3d精匹配器
    ceres_scan_matcher_3d = {
      occupied_space_weight_0 = 5.,
      occupied_space_weight_1 = 30.,
      translation_weight = 10.,
      rotation_weight = 1.,
      only_optimize_yaw = false,
      ceres_solver_options = {
        use_nonmonotonic_steps = false,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
  },

  matcher_translation_weight = 5e2, -- 当前submap与在当前submap内的某个node的之间的平移约束
  matcher_rotation_weight = 1.6e3, -- 当前submap与在当前submap内的某个node的之间的旋转约束

  -- 优化残差方程的相关参数
  optimization_problem = {
    huber_scale = 1e1,                -- 值越大,（潜在）异常值的影响就越大 Huber因子越大，离群值（错误的数据）对整体的影响越大。
    acceleration_weight = 1.1e2,      -- 3d中imu的线加速度的权重
    rotation_weight = 1.6e4,          -- 3d中imu的旋转的权重
    
    -- 前端结果残差的权重
    local_slam_pose_translation_weight = 1e5, -- 前后2个node之间局部观测与全局优化之间的平移约束权重
    local_slam_pose_rotation_weight = 1e5, -- 前后两个node之间的局部观测与全局优化之间旋转约束权重
    -- 里程计残差的权重
    odometry_translation_weight = 1e5, --前后两个node之间的局部观测与里程计观测之间的平移约束权重
    odometry_rotation_weight = 1e5, --前后两个node之间的局部观测与里程计观测之间的旋转约束权重
    -- gps残差的权重
    fixed_frame_pose_translation_weight = 1e1,
    fixed_frame_pose_rotation_weight = 1e2,
    fixed_frame_pose_use_tolerant_loss = false,
    fixed_frame_pose_tolerant_loss_param_a = 1,
    fixed_frame_pose_tolerant_loss_param_b = 1,

    log_solver_summary = false, -- 是否打印ceres优化后的结果
    use_online_imu_extrinsics_in_3d = true, -- 是否在线标定imu的外参
    fix_z_in_3d = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 50,
      num_threads = 7,
    },
  },

  max_num_final_iterations = 200,   -- 在建图结束之后执行一次全局优化, 不要求实时性, 迭代次数多 最终优化次数
  global_sampling_ratio = 0.003,    -- 纯定位时候查找回环的频率 全局地图匹配约束nodes采样比率，间隔1/0.003个node进行一次全局约束检测
  log_residual_histograms = true, -- 是否输出残差直方图

  -- 间隔多久全局匹配一次，这个要和global_sampling_ratio同时满足要求，才进行全局地图匹配寻找回环，这个参数在多机器人同时建图中使用。
  global_constraint_search_after_n_seconds = 10., -- 纯定位时多少秒执行一次全子图的约束计算 

  --  overlapping_submaps_trimmer_2d = {
  --    fresh_submaps_count = 1,
  --    min_covered_area = 2,
  --    min_added_submaps_count = 5,
  --  },
}
