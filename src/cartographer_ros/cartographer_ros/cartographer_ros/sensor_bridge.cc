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

#include "cartographer_ros/sensor_bridge.h"

#include "absl/memory/memory.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

namespace {

// 检查frame_id是否带有'/',如果带则报错
const std::string& CheckNoLeadingSlash(const std::string& frame_id) {
  if (frame_id.size() > 0) {
    CHECK_NE(frame_id[0], '/') << "The frame_id " << frame_id
                               << " should not start with a /. See 1.7 in "
                                  "http://wiki.ros.org/tf2/Migration.";
  }
  return frame_id;
}

}  // namespace

/**
 * @brief 构造函数, 并且初始化TfBridge
 * 
 * @param[in] num_subdivisions_per_laser_scan 一帧数据分成几次发送
 * @param[in] tracking_frame 数据都转换到tracking_frame
 * @param[in] lookup_transform_timeout_sec 查找tf的超时时间
 * @param[in] tf_buffer tf_buffer
 * @param[in] trajectory_builder 轨迹构建器
 */
SensorBridge::SensorBridge(
    const int num_subdivisions_per_laser_scan,
    const std::string& tracking_frame,
    const double lookup_transform_timeout_sec, 
    tf2_ros::Buffer* const tf_buffer,
    // 父类指针接收指向子类的指针
    carto::mapping::TrajectoryBuilderInterface* const trajectory_builder)
    : num_subdivisions_per_laser_scan_(num_subdivisions_per_laser_scan),
      // 构造函数初始化
      tf_bridge_(tracking_frame, lookup_transform_timeout_sec, tf_buffer),
      trajectory_builder_(trajectory_builder) {}

// 将ros格式的里程计数据(pose) 转成tracking frame的pose, 再转成carto的里程计数据类型
std::unique_ptr<carto::sensor::OdometryData> SensorBridge::ToOdometryData(
    const nav_msgs::Odometry::ConstPtr& msg)
{
  // from ros to icu's time
  const carto::common::Time time = FromRos(msg->header.stamp);

  // 返回child_frame_id(base_footprint、base_link) 到 tracking(imu_link)的坐标变换：T_footprint2imu
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
      time, CheckNoLeadingSlash(msg->child_frame_id));
  if (sensor_to_tracking == nullptr) 
  {
    return nullptr;
  }

  // 将里程计footprint的pose转成tracking_frame的pose, 再转成carto的里程计数据类型
  return absl::make_unique<carto::sensor::OdometryData>(
      // 将数据转换到imu坐标系下： T_footprint2imu¯¹ * pose_footprint = T_imu2footprint * pose_footprint = pose_imu
      carto::sensor::OdometryData{
          time, ToRigid3d(msg->pose.pose) * sensor_to_tracking->inverse()});// 重载*
}

// 调用trajectory_builder_的AddSensorData进行数据的处理
void SensorBridge::HandleOdometryMessage(
    const std::string& sensor_id, const nav_msgs::Odometry::ConstPtr& msg) 
{
  // 将ros格式的里程计数据(pose) 转成tracking frame的pose, 再转成carto的里程计数据类型
  std::unique_ptr<carto::sensor::OdometryData> odometry_data = ToOdometryData(msg);

  if (odometry_data != nullptr)
  {
    trajectory_builder_->AddSensorData(sensor_id,
        carto::sensor::OdometryData{odometry_data->time, odometry_data->pose});
  }
}

// 将ros格式的gps数据转换成相对坐标系下的坐标,再调用trajectory_builder_的AddSensorData进行数据的处理
void SensorBridge::HandleNavSatFixMessage(
    const std::string& sensor_id, const sensor_msgs::NavSatFix::ConstPtr& msg) 
{

  const carto::common::Time time = FromRos(msg->header.stamp);
  // 如果不是固定解,就加入一个固定的空位姿
  if (msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) 
  {
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::FixedFramePoseData{time, absl::optional<Rigid3d>()});// 空位姿
    return;
  }
  // STATUS_FIX：GPS不能有跳变值(过隧道)，协方差不能太大，都会使得带偏轨迹

  // 确定ecef原点到局部坐标系的坐标变换
  if (!ecef_to_local_frame_.has_value()) 
  {
    // T_ecef2local
    ecef_to_local_frame_ = ComputeLocalFrameFromLatLong(msg->latitude, msg->longitude);
    LOG(INFO) << "Using NavSatFix. Setting ecef_to_local_frame with lat = "
              << msg->latitude << ", long = " << msg->longitude << ".";
  }

  // 通过这个坐标变换 乘以 之后的gps数据,就相当于减去了一个固定的坐标, 从而得到了gps数据间的相对坐标变换 notice:
  // 先将数据转到第一帧坐标系下,再调用trajectory_builder_的AddSensorData进行数据的处理
  trajectory_builder_->AddSensorData(
      sensor_id,
      carto::sensor::FixedFramePoseData{time,
      // 在第一帧数据的坐标系下，其他帧的坐标：P_local = T_ecef2local* P_ecef(GPS数据转换成的地球坐标系下的坐标)
      absl::optional<Rigid3d>(Rigid3d::Translation(ecef_to_local_frame_.value() *
                               LatLongAltToEcef(msg->latitude, msg->longitude,msg->altitude)))});// 重载*:RP+t
}

// 处理Landmark数据, 先转成carto的格式,再传入trajectory_builder_
void SensorBridge::HandleLandmarkMessage(
    const std::string& sensor_id,
    const cartographer_ros_msgs::LandmarkList::ConstPtr& msg) 
{
  // 将在ros中自定义的LandmarkList类型的数据, 转成carto的消息类型LandmarkData
  auto landmark_data = ToLandmarkData(*msg);

  // 获取 landmark的frame(fram_id)到tracking_frame的坐标变换：T_mark2imu
  auto tracking_from_landmark_sensor = tf_bridge_.LookupToTracking(
      landmark_data.time, CheckNoLeadingSlash(msg->header.frame_id));

  // 遍历每一个地标数据，将数据转到tracking_frame下
  if (tracking_from_landmark_sensor != nullptr) 
  { 
    for (auto& observation : landmark_data.landmark_observations) 
    {
      // T_imu = T_imu2mark * T_mark
      observation.landmark_to_tracking_transform =
          *tracking_from_landmark_sensor * observation.landmark_to_tracking_transform;
    }
  }
  // 调用trajectory_builder_的AddSensorData进行数据的处理
  trajectory_builder_->AddSensorData(sensor_id, landmark_data);
}

// 进行数据类型转换与坐标系的转换
std::unique_ptr<carto::sensor::ImuData> SensorBridge::ToImuData(
    const sensor_msgs::Imu::ConstPtr& msg) 
{
  // 检查是否存在线性加速度与角速度
  CHECK_NE(msg->linear_acceleration_covariance[0], -1)
      << "Your IMU data claims to not contain linear acceleration measurements "
         "by setting linear_acceleration_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";
  CHECK_NE(msg->angular_velocity_covariance[0], -1)
      << "Your IMU data claims to not contain angular velocity measurements "
         "by setting angular_velocity_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";

  const carto::common::Time time = FromRos(msg->header.stamp);
  // 
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
      time, CheckNoLeadingSlash(msg->header.frame_id));
  if (sensor_to_tracking == nullptr) 
  {
    return nullptr;
  }
  // 推荐将imu的坐标系当做tracking frame，因为IMU的频率很高，不适合坐标变换(计算量大)
  CHECK(sensor_to_tracking->translation().norm() < 1e-5)
      << "The IMU frame must be colocated with the tracking frame. "
         "Transforming linear acceleration into the tracking frame will "
         "otherwise be imprecise.";
  // 结构体初始化进行：将IMU数据转换到IMU坐标系下，旋转矩阵为单位阵，相当于没有旋转
  return absl::make_unique<carto::sensor::ImuData>(carto::sensor::ImuData{
      time, sensor_to_tracking->rotation() * ToEigen(msg->linear_acceleration),
      sensor_to_tracking->rotation() * ToEigen(msg->angular_velocity)});
}

// 调用trajectory_builder_的AddSensorData进行数据的处理
void SensorBridge::HandleImuMessage(const std::string& sensor_id,
                                    const sensor_msgs::Imu::ConstPtr& msg) 
{
  // 将IMU数据转换到IMU坐标系下，转换的数据是角加速度和线加速度
  std::unique_ptr<carto::sensor::ImuData> imu_data = ToImuData(msg);
  if (imu_data != nullptr) 
  {
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::ImuData{imu_data->time, imu_data->linear_acceleration,
                               imu_data->angular_velocity});
  }
}

// 重载函数处理LaserScan数据, 先转成点云,再传入trajectory_builder_
// 入参：传感器id、ros格式数据
void SensorBridge::HandleLaserScanMessage(
    const std::string& sensor_id, const sensor_msgs::LaserScan::ConstPtr& msg) 
{
  // 带强度的激光点云数据：坐标与时间容器、强度容器
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  // 把ros格式的数据msg转换成点云数据，返回这一帧数据的点云坐标，以及一帧激光的总时长
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);// 函数重载
  // 处理点云数据，入参：传感器id、一帧激光的总时长、frame_id、点云
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}

// 处理MultiEchoLaserScan数据, 先转成点云,再传入trajectory_builder_
void SensorBridge::HandleMultiEchoLaserScanMessage(
    const std::string& sensor_id,
    const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) 
{
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  // 返回这一帧数据的点云坐标，以及一帧激光的总时长
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);// 函数重载
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}

// 处理ros格式的PointCloud2, 先转成点云,再传入trajectory_builder_
void SensorBridge::HandlePointCloud2Message(
    const std::string& sensor_id,
    const sensor_msgs::PointCloud2::ConstPtr& msg) 
{
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);// 函数重载
  HandleRangefinder(sensor_id, time, msg->header.frame_id, point_cloud.points);
}

const TfBridge& SensorBridge::tf_bridge() const { return tf_bridge_; }


// 根据参数配置,将一帧雷达数据分成几段, 再传入trajectory_builder_
// 入参：传感器id、一帧激光的总时长、frame_id、点云坐标
void SensorBridge::HandleLaserScan(
    const std::string& sensor_id, 
    const carto::common::Time time,
    const std::string& frame_id,
    const carto::sensor::PointCloudWithIntensities& points) 
{
  if (points.points.empty()) 
  {
    return;
  }
  // CHECK_LE(less、equ): 点云最后一个点的相对时间戳小于等于0
  CHECK_LE(points.points.back().time, 0.f);
  // TODO(gaschler): Use per-point time instead of subdivisions.

  // 意为一帧雷达数据被分成几次处理(防止一帧激光数据点太多，方便处理点云畸变), 一般将这个参数设置为1
  for (int i = 0; i != num_subdivisions_per_laser_scan_; ++i) 
  {
    const size_t start_index =// no1:100*0/2 = 0, no2:100*1/2 = 50 
        points.points.size() * i / num_subdivisions_per_laser_scan_;
    const size_t end_index =// no1:100*1/2 = 50, no2:100*2/2 = 100
        points.points.size() * (i + 1) / num_subdivisions_per_laser_scan_;
    
    // 生成分段后的点云(0~50)，对应的时间(-100~-49)
    carto::sensor::TimedPointCloud subdivision(
        points.points.begin() + start_index, points.points.begin() + end_index);
    if (start_index == end_index) 
    {
      continue;
    }
    // no1:一小段激光最后一个点(第50个点)的时间戳(负的或0)，如-50 ，no2:0
    const double time_to_subdivision_end = subdivision.back().time;
    // `subdivision_time` is the end of the measurement so sensor::Collator will
    // send all other sensor data first.
    // no1:time一帧激光的总时长(100) ，subdivision_time(正的) = 100+(-50) = 50
    const carto::common::Time subdivision_time =
        time + carto::common::FromSeconds(time_to_subdivision_end);
    
    // sensor_id看做是scan，上一次子段的时间戳，it是std::map<std::string, cartographer::common::Time>
    auto it = sensor_to_previous_subdivision_time_.find(sensor_id);
    if (it != sensor_to_previous_subdivision_time_.end() &&
        // 上一段点云的时间大于等于这一段点云的时间，数据重叠
        // 这一帧的数据还没处理完，又来了新的数据，应该把新的数据的头部分布忽略
        // no2:50 >= 100 fasle:
        it->second >= subdivision_time)
    {
      LOG(WARNING) << "Ignored subdivision of a LaserScan message from sensor "
                   << sensor_id << " because previous subdivision time "
                   << it->second << " is not before current subdivision time "
                   << subdivision_time;
      continue;// 下一小块
    }
    // 更新对应sensor_id的时间戳 no1:50
    sensor_to_previous_subdivision_time_[sensor_id] = subdivision_time;
    
    // 遍历小块中的点
    for (auto& point : subdivision)
    {
      // 相对于本小段内的最后一个点的时间 = 0，no1:一帧点云的时间-(-50)
      point.time -= time_to_subdivision_end;
    }
    // 小块中最后一个点的时间为0
    CHECK_EQ(subdivision.back().time, 0.f);
    // notice: 将分段后的点云 subdivision 传入 trajectory_builder_
    HandleRangefinder(sensor_id, subdivision_time, frame_id, subdivision);
  } // for 
}

// 雷达相关的数据最终的处理函数
// 先将数据转到tracking坐标系下,再调用trajectory_builder_的AddSensorData进行数据的处理

/**
 * @brief 
 * 
 * @param[in] sensor_id 数据的话题，如scan
 * @param[in] time 点云的时间戳(最后一个点的时间)
 * @param[in] frame_id 点云的frame
 * @param[in] ranges 雷达坐标系下的TimedPointCloud格式的点云
 */
void SensorBridge::HandleRangefinder(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id, const carto::sensor::TimedPointCloud& ranges) 
{
  if (!ranges.empty()) 
  {
    // 每个小段的最后一个时间戳应该小于等于0
    CHECK_LE(ranges.back().time, 0.f);
  }
  // 雷达坐标系下的数据转换到tracking坐标系下
  const auto sensor_to_tracking =
      tf_bridge_.LookupToTracking(time, CheckNoLeadingSlash(frame_id));

  // 以 tracking 到 sensor_frame 的坐标变换为TimedPointCloudData 的 origin
  // 将点云的坐标转成 tracking 坐标系下的坐标, 再传入trajectory_builder_
  if (sensor_to_tracking != nullptr) 
  {
    // trajectory_builder_ = map_builder_->GetTrajectoryBuilder(trajectory_id)); 
    trajectory_builder_->AddSensorData(sensor_id,
        // notice: 雷达坐标系在IMU坐标系下的位置坐标 Vector3f origin = sensor_to_tracking->translation() = tracking坐标系下雷达坐标系的原点
        carto::sensor::TimedPointCloudData{time, sensor_to_tracking->translation().cast<float>(),
        // 将点云从雷达坐标系下转到tracking_frame坐标系系下：T*Point
        carto::sensor::TransformTimedPointCloud(ranges, sensor_to_tracking->cast<float>())} ); // 强度始终为空
  }
}

}  // namespace cartographer_ros
