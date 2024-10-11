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
  map_builder = MAP_BUILDER,               -- map_builder.lua的配置信息
  trajectory_builder = TRAJECTORY_BUILDER, -- trajectory_builder.lua的配置信息
  map_frame = "map",                       -- 地图坐标系的名称
  tracking_frame = "base_link",            -- 将所有传感器数据转换到这个坐标系下
  published_frame = "base_link",           -- bag的最顶层坐标系，cart会把此作为map的子坐标系
  odom_frame = "odom",                     -- 里程计坐标系的名称
  provide_odom_frame = true,               -- 是否提供里程计的tf，有则map->odom->bask_link
  publish_frame_projected_to_2d = false,   -- 是否将坐标系投影到平面上
  use_pose_extrapolator = true,            -- 发布tf时使用pose_extrapolator预测的位姿还是前端计算的位姿
  use_odometry = false,                    -- 是否使用里程计，如果使用，则一定要有odom的tf
  use_nav_sat = false,                     -- 是否使用gps
  use_landmarks = false,                   -- 是否使用landmark
  num_laser_scans = 0,                     -- 是否使用单线激光雷达
  num_multi_echo_laser_scans = 1,          -- 是否使用muti_echo_laser_sacns数据
  num_subdivisions_per_laser_scan = 10,    -- 一帧数据被分为几次处理，一般为1
  num_point_clouds = 0,                    -- 是否使用点云数据
  lookup_transform_timeout_sec = 0.2,      -- 查找tf的超时时间
  submap_publish_period_sec = 0.3,         -- 发布地图数据的时间间隔
  pose_publish_period_sec = 5e-3,          -- 发布位姿数据的时间间隔
  trajectory_publish_period_sec = 30e-3,   -- 发布轨迹数据的时间间隔
  rangefinder_sampling_ratio = 1.,         -- 传感器的采样频率，1表示为传感器发布频率，0.5则为一半
  odometry_sampling_ratio = 1.,            -- 里程计的采样频率
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10

return options
