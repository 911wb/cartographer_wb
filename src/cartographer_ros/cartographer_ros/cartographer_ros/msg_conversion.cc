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

#include "cartographer_ros/msg_conversion.h"

#include <cmath>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/transform/proto/transform.pb.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/time_conversion.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Vector3.h"
#include "glog/logging.h"
#include "nav_msgs/OccupancyGrid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "ros/serialization.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/PointCloud2.h"

namespace {

// Sizes of PCL point types have to be 4n floats for alignment, as described in
// http://pointclouds.org/documentation/tutorials/adding_custom_ptype.php
// 自定义的pcl点云格式
struct PointXYZT {
  float x;
  float y;
  float z;
  float time;
};

struct PointXYZIT {
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  float unused_padding[2];
};

}  // namespace

// 将自定义的点云格式在pcl中注册
POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZT, (float, x, x)(float, y, y)(float, z, z)(float, time, time))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(float, time, time))

namespace cartographer_ros {
namespace {

// The ros::sensor_msgs::PointCloud2 binary data contains 4 floats for each
// point. The last one must be this value or RViz is not showing the point cloud
// properly.
constexpr float kPointCloudComponentFourMagic = 1.;

using ::cartographer::sensor::LandmarkData;
using ::cartographer::sensor::LandmarkObservation;
using ::cartographer::sensor::PointCloudWithIntensities;
using ::cartographer::transform::Rigid3d;
using ::cartographer_ros_msgs::LandmarkEntry;
using ::cartographer_ros_msgs::LandmarkList;

/**
 * @brief 点云格式的设置与数组的初始化
 * 
 * @param[in] timestamp 时间戳
 * @param[in] frame_id 坐标系
 * @param[in] num_points 点云的个数
 * @return sensor_msgs::PointCloud2 
 */
sensor_msgs::PointCloud2 PreparePointCloud2Message(const int64_t timestamp,
                                                   const std::string& frame_id,
                                                   const int num_points) {
  sensor_msgs::PointCloud2 msg;
  msg.header.stamp = ToRos(::cartographer::common::FromUniversal(timestamp));
  msg.header.frame_id = frame_id;
  msg.height = 1;
  msg.width = num_points;
  msg.fields.resize(3);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.is_bigendian = false;
  msg.point_step = 16;
  msg.row_step = 16 * msg.width;
  msg.is_dense = true;
  msg.data.resize(16 * num_points);
  return msg;
}

// For sensor_msgs::LaserScan.
bool HasEcho(float) { return true; }

float GetFirstEcho(float range) { return range; }

// For sensor_msgs::MultiEchoLaserScan.
bool HasEcho(const sensor_msgs::LaserEcho& echo) {
  return !echo.echoes.empty();
}

// 通过函数重载, 使得函数可以同时适用LaserScan与LaserEcho
float GetFirstEcho(const sensor_msgs::LaserEcho& echo) {
  return echo.echoes[0];
}

// 函数模板 == 编译器自动帮你实现函数重载
// 将LaserScan 或者 MultiEchoLaserScan转成carto格式的点云数据
template <typename LaserMessageType>
std::tuple<PointCloudWithIntensities, ::cartographer::common::Time>
LaserScanToPointCloudWithIntensities(const LaserMessageType& msg) 
{
  // 最短距离要大于0
  CHECK_GE(msg.range_min, 0.f);
  CHECK_GE(msg.range_max, msg.range_min);
  // 激光点云之间的角度增量
  if (msg.angle_increment > 0.f) 
  {
    // 最后一个点的角度、第一个点的角度
    CHECK_GT(msg.angle_max, msg.angle_min);
  } 
  else 
  {
    CHECK_GT(msg.angle_min, msg.angle_max);
  }
  // 创建容器，下面填充，带有强度和时间信息的点云数据
  PointCloudWithIntensities point_cloud;
  // 每一个点的角度，初始化为第一个点的角度
  float angle = msg.angle_min;

  // 遍历每一个点，激光数据探测的是距离值，而不是坐标值
  for (size_t i = 0; i < msg.ranges.size(); ++i) 
  {
    // c++11: 使用auto可以适应不同的数据类型
    // 一束激光发射出去有多个返回值，取最强的返回值计算探测距离
    // ranges有两种类型：float32[]、LaserEcho[]
    const auto& echoes = msg.ranges[i];

    // 重载两次HasEcho：HasEcho(float)、HasEcho(LaserEcho)
    if (HasEcho(echoes))
    {
      // 获取首次回波或雷达数据，重载两次GetFirstEcho
      const float first_echo = GetFirstEcho(echoes);

      // 满足范围才进行使用 ，探测距离在特定的范围内，把距离值，经过角度转换为坐标值
      if (msg.range_min <= first_echo && first_echo <= msg.range_max) 
      {
        // 轴角：绕着z轴转动
        const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
        // 结构体：点云xyz坐标，及其对应的时间
        const cartographer::sensor::TimedRangefinderPoint point{
            // position = rotation * 测距值 * x轴的单位向量
            rotation * (first_echo * Eigen::Vector3f::UnitX()),
            // time 点和点之间的时间差，遍历完后，最后一个点的时间为一帧激光的总时长
            i * msg.time_increment};
        // 保存点云坐标与时间信息
        point_cloud.points.push_back(point);
        
        // 如果存在强度信息
        if (msg.intensities.size() > 0)
        {
          CHECK_EQ(msg.intensities.size(), msg.ranges.size());
          // 使用auto可以适应不同的数据类型
          const auto& echo_intensities = msg.intensities[i];
          CHECK(HasEcho(echo_intensities));
          point_cloud.intensities.push_back(GetFirstEcho(echo_intensities));
        } 
        else 
        {
          point_cloud.intensities.push_back(0.f);
        }
      }
    }
    angle += msg.angle_increment;
  }
  // 第一个点的时间戳
  ::cartographer::common::Time timestamp = FromRos(msg.header.stamp);
  if (!point_cloud.points.empty()) 
  {
    // 最后一个点的时间，即一帧激光的总时长
    const double duration = point_cloud.points.back().time;
    // 点云开始的时间 加上 第一个点到最后一个点的时间
    // 以点云最后一个点的时间为点云的时间戳，每一帧测量的时间看作是最后一个点的时间戳
    timestamp += cartographer::common::FromSeconds(duration);

    // 让点云的时间变成相对值, 最后一个点的时间为0，前面的点的时间都是负值(一开始是第一个点云的时间为0)
    for (auto& point : point_cloud.points) 
    {
      point.time -= duration;
    }
  }
  // 返回这一帧数据的点云坐标，以及一帧激光的总时长(最后一个点的时间)
  return std::make_tuple(point_cloud, timestamp);
}

// 检查点云是否存在 field_name 字段
bool PointCloud2HasField(const sensor_msgs::PointCloud2& pc2,
                         const std::string& field_name) {
  for (const auto& field : pc2.fields) {
    if (field.name == field_name) {
      return true;
    }
  }
  return false;
}

}  // namespace

/**
 * @brief 将cartographer格式的点云数据 转换成 ROS格式的点云数据
 *
 * @param[in] timestamp 时间戳
 * @param[in] frame_id 坐标系
 * @param[in] point_cloud 地图坐标系下的点云数据
 * @return sensor_msgs::PointCloud2 ROS格式的点云数据
 */
sensor_msgs::PointCloud2 ToPointCloud2Message(
    const int64_t timestamp, const std::string& frame_id,
    const ::cartographer::sensor::TimedPointCloud& point_cloud) 
{
  // 点云格式的设置与数组的初始化
  auto msg = PreparePointCloud2Message(timestamp, frame_id, point_cloud.size());

  // note: 通过ros::serialization将msg放进内存中
  ::ros::serialization::OStream stream(msg.data.data(), msg.data.size());
  for (const cartographer::sensor::TimedRangefinderPoint& point : point_cloud) {
    // 通过使用next()函数,将点的坐标 序列化 到stream输出流, 将point存入msg
    stream.next(point.position.x());
    stream.next(point.position.y());
    stream.next(point.position.z());
    stream.next(kPointCloudComponentFourMagic); // kPointCloudComponentFourMagic = 1
  }
  return msg;
}

// 由ros格式的LaserScan转成carto格式的PointCloudWithIntensities
std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::LaserScan& msg)// 函数重载 
{
  // 返回这一帧数据的点云坐标，以及一帧激光的总时长
  return LaserScanToPointCloudWithIntensities(msg);// 函数模板
}

// 由ros格式的MultiEchoLaserScan转成carto格式的PointCloudWithIntensities
std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::MultiEchoLaserScan& msg) // 函数重载
{
  // 返回这一帧数据的点云坐标，以及一帧激光的总时长
  return LaserScanToPointCloudWithIntensities(msg);// 函数模板
}

// 由ros格式的PointCloud2转成carto格式的PointCloudWithIntensities
std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::PointCloud2& msg) // 函数重载
{

  PointCloudWithIntensities point_cloud;
  // We check for intensity field here to avoid run-time warnings if we pass in
  // a PointCloud2 without intensity.

  // 有强度数据
  if (PointCloud2HasField(msg, "intensity"))
  {
    // 有强度字段, 有时间字段
    if (PointCloud2HasField(msg, "time"))
    {
      // 创建容器
      pcl::PointCloud<PointXYZIT> pcl_point_cloud;
      // 填充容器
      pcl::fromROSMsg(msg, pcl_point_cloud);
      point_cloud.points.reserve(pcl_point_cloud.size());
      point_cloud.intensities.reserve(pcl_point_cloud.size());
      // 遍历容器
      for (const auto& point : pcl_point_cloud) 
      {
        point_cloud.points.push_back(
            {Eigen::Vector3f{point.x, point.y, point.z}, point.time});
        point_cloud.intensities.push_back(point.intensity);
      }
    } 
    // 有强度字段, 没时间字段
    else 
    {
      pcl::PointCloud<pcl::PointXYZI> pcl_point_cloud;
      pcl::fromROSMsg(msg, pcl_point_cloud);
      point_cloud.points.reserve(pcl_point_cloud.size());
      point_cloud.intensities.reserve(pcl_point_cloud.size());
      for (const auto& point : pcl_point_cloud) 
      {
        point_cloud.points.push_back(
            {Eigen::Vector3f{point.x, point.y, point.z}, 0.f}); // 没有时间信息就把时间填0
        point_cloud.intensities.push_back(point.intensity);
      }
    }
  }
  // 没有强度数据
  else {
    // If we don't have an intensity field, just copy XYZ and fill in 1.0f.
    // 没强度字段, 有时间字段
    if (PointCloud2HasField(msg, "time")) {
      pcl::PointCloud<PointXYZT> pcl_point_cloud;
      pcl::fromROSMsg(msg, pcl_point_cloud);
      point_cloud.points.reserve(pcl_point_cloud.size());
      point_cloud.intensities.reserve(pcl_point_cloud.size());
      for (const auto& point : pcl_point_cloud) {
        point_cloud.points.push_back(
            {Eigen::Vector3f{point.x, point.y, point.z}, point.time});
        point_cloud.intensities.push_back(1.0f);// 没有强度
      }
    } 
    // 没强度字段, 没时间字段
    else {
      pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
      pcl::fromROSMsg(msg, pcl_point_cloud);
      point_cloud.points.reserve(pcl_point_cloud.size());
      point_cloud.intensities.reserve(pcl_point_cloud.size());
      for (const auto& point : pcl_point_cloud) {
        point_cloud.points.push_back(
            {Eigen::Vector3f{point.x, point.y, point.z}, 0.f}); // 没有时间信息就把时间填0
        point_cloud.intensities.push_back(1.0f);// 没有强度
      }
    }
  }

  ::cartographer::common::Time timestamp = FromRos(msg.header.stamp);
  if (!point_cloud.points.empty()) 
  {
    // 一帧激光的总时长
    const double duration = point_cloud.points.back().time;
    // 点云最后一个点的时间 作为整个点云的时间戳
    timestamp += cartographer::common::FromSeconds(duration);

    for (auto& point : point_cloud.points) 
    {
      // 将每个点的时间减去整个点云的时间, 所以每个点的时间都应该小于0
      point.time -= duration;

      // 对每个点进行时间检查, 看是否有数据点的时间大于0, 大于0就报错
      CHECK_LE(point.time, 0.f)
          << "Encountered a point with a larger stamp than "
             "the last point in the cloud.";
    }
  }
  // 不管是laserscan、多回声波雷达、点云，都返回最后一个点的时间戳作为整帧的时间戳
  return std::make_tuple(point_cloud, timestamp);
}

  // 将在ros中自定义的LandmarkList类型的数据, 转成carto的消息类型LandmarkData
LandmarkData ToLandmarkData(const LandmarkList& landmark_list) 
{
  LandmarkData landmark_data;
  // from ros to icu
  landmark_data.time = FromRos(landmark_list.header.stamp);
  // 遍历每个地标
  for (const LandmarkEntry& entry : landmark_list.landmarks) 
  {
    // 添加数据
    landmark_data.landmark_observations.push_back(
        {entry.id, ToRigid3d(entry.tracking_from_landmark_transform),// T_mark2imu
         entry.translation_weight, entry.rotation_weight});
  }
  return landmark_data;
}

Rigid3d ToRigid3d(const geometry_msgs::TransformStamped& transform) {
  return Rigid3d(ToEigen(transform.transform.translation),
                 ToEigen(transform.transform.rotation));
}

Rigid3d ToRigid3d(const geometry_msgs::Pose& pose) {
  return Rigid3d({pose.position.x, pose.position.y, pose.position.z},
                 ToEigen(pose.orientation));
}

Eigen::Vector3d ToEigen(const geometry_msgs::Vector3& vector3) {
  return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
}

Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion& quaternion) {
  return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y,
                            quaternion.z);
}

geometry_msgs::Transform ToGeometryMsgTransform(const Rigid3d& rigid3d) {
  geometry_msgs::Transform transform;
  transform.translation.x = rigid3d.translation().x();
  transform.translation.y = rigid3d.translation().y();
  transform.translation.z = rigid3d.translation().z();
  transform.rotation.w = rigid3d.rotation().w();
  transform.rotation.x = rigid3d.rotation().x();
  transform.rotation.y = rigid3d.rotation().y();
  transform.rotation.z = rigid3d.rotation().z();
  return transform;
}

/**
 * @brief 将cartographer的Rigid3d位姿格式,转换成ROS的 geometry_msgs::Pose 格式
 * 
 * @param[in] rigid3d Rigid3d格式的位姿
 * @return geometry_msgs::Pose ROS格式的位姿
 */
geometry_msgs::Pose ToGeometryMsgPose(const Rigid3d& rigid3d) {
  geometry_msgs::Pose pose;
  pose.position = ToGeometryMsgPoint(rigid3d.translation());
  pose.orientation.w = rigid3d.rotation().w();
  pose.orientation.x = rigid3d.rotation().x();
  pose.orientation.y = rigid3d.rotation().y();
  pose.orientation.z = rigid3d.rotation().z();
  return pose;
}

geometry_msgs::Point ToGeometryMsgPoint(const Eigen::Vector3d& vector3d) {
  geometry_msgs::Point point;
  point.x = vector3d.x();
  point.y = vector3d.y();
  point.z = vector3d.z();
  return point;
}

// 将经纬度数据转换成ecef坐标系下的坐标
Eigen::Vector3d LatLongAltToEcef(const double latitude, const double longitude,
                                 const double altitude) 
{
  // note: 地固坐标系(Earth-Fixed Coordinate System)也称地球坐标系, 
  // 是固定在地球上与地球一起旋转的坐标系.
  // 如果忽略地球潮汐和板块运动, 地面上点的坐标值在地固坐标系中是固定不变的.

  // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates
  
  constexpr double a = 6378137.;  // semi-major axis, equator to center.
  constexpr double f = 1. / 298.257223563;
  constexpr double b = a * (1. - f);  // semi-minor axis, pole to center.
  constexpr double a_squared = a * a;
  constexpr double b_squared = b * b;
  constexpr double e_squared = (a_squared - b_squared) / a_squared;
  const double sin_phi = std::sin(cartographer::common::DegToRad(latitude));
  const double cos_phi = std::cos(cartographer::common::DegToRad(latitude));
  const double sin_lambda = std::sin(cartographer::common::DegToRad(longitude));
  const double cos_lambda = std::cos(cartographer::common::DegToRad(longitude));
  const double N = a / std::sqrt(1 - e_squared * sin_phi * sin_phi);
  const double x = (N + altitude) * cos_phi * cos_lambda;
  const double y = (N + altitude) * cos_phi * sin_lambda;
  const double z = (b_squared / a_squared * N + altitude) * sin_phi;

  return Eigen::Vector3d(x, y, z);
}

/**
 * @brief 计算第一帧GPS数据指向ECEF坐标系下原点的坐标变换, 用这个坐标变换乘以之后的GPS数据
 * 就得到了之后的GPS数据相对于第一帧GPS数据的相对坐标变换
 * @param[in] latitude 维度数据
 * @param[in] longitude 经度数据
 * @return cartographer::transform::Rigid3d 计算第一帧GPS数据指向ECEF坐标系下原点的坐标变换
 */
cartographer::transform::Rigid3d ComputeLocalFrameFromLatLong(
    const double latitude, const double longitude)
{
  // 将经度和纬度转换成第一帧GPS数据的坐标，以地球中心为原点的坐标系，P_ecef
  const Eigen::Vector3d translation = LatLongAltToEcef(latitude, longitude, 0.);
  // 将经度和纬度转换成四元数，R_first2ecef
  const Eigen::Quaterniond rotation =
      Eigen::AngleAxisd(cartographer::common::DegToRad(latitude - 90.), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(cartographer::common::DegToRad(-longitude), Eigen::Vector3d::UnitZ());

  // 第一帧GPS数据 到 ECEF坐标系下原点的坐标变换：T_fist2ecef， Rigid3<double>() 构造函数初始化
  return cartographer::transform::Rigid3d(rotation * -translation, rotation);
}

/**
 * @brief 由cairo的图像生成ros格式的地图
 * 
 * @param[in] painted_slices 
 * @param[in] resolution 栅格地图的分辨率
 * @param[in] frame_id 栅格地图的坐标系
 * @param[in] time 地图对应的时间
 * @return std::unique_ptr<nav_msgs::OccupancyGrid> ros格式的栅格地图
 */
std::unique_ptr<nav_msgs::OccupancyGrid> CreateOccupancyGridMsg(
    const cartographer::io::PaintSubmapSlicesResult& painted_slices,
    const double resolution, const std::string& frame_id,
    const ros::Time& time) 
{

  auto occupancy_grid = absl::make_unique<nav_msgs::OccupancyGrid>();

  const int width = cairo_image_surface_get_width(painted_slices.surface.get());
  const int height =
      cairo_image_surface_get_height(painted_slices.surface.get());

  occupancy_grid->header.stamp = time;
  // 
  occupancy_grid->header.frame_id = frame_id;
  
  occupancy_grid->info.map_load_time = time;
  occupancy_grid->info.resolution = resolution;
  occupancy_grid->info.width = width;
  occupancy_grid->info.height = height;
  // position_origin = (x,y,z)
  // 
  occupancy_grid->info.origin.position.x = -painted_slices.origin.x() * resolution;
  // carto:坐标原点在左上角 , ros坐标原点:左下角
  occupancy_grid->info.origin.position.y = (-height + painted_slices.origin.y()) * resolution;
  occupancy_grid->info.origin.position.z = 0.;
  // R_origin = I
  occupancy_grid->info.origin.orientation.w = 1.;
  occupancy_grid->info.origin.orientation.x = 0.;
  occupancy_grid->info.origin.orientation.y = 0.;
  occupancy_grid->info.origin.orientation.z = 0.;

  // 获取 uint32_t* 格式的地图数据
  const uint32_t* pixel_data = reinterpret_cast<uint32_t*>(
      cairo_image_surface_get_data(painted_slices.surface.get()));
      
  occupancy_grid->data.reserve(width * height);

  // 遍历cairo的图像中的每个像素
  for (int y = height - 1; y >= 0; --y) 
  {
    for (int x = 0; x < width; ++x) 
    {
      const uint32_t packed = pixel_data[y * width + x];
      
      // 根据packed获取像素值[0-255]，栅格值intensity_value
      const unsigned char color = packed >> 16;
      // 根据packed获取这个栅格是否被更新过,observed
      const unsigned char observed = packed >> 8;

      // source code
      // 根据像素值确定栅格占用值
      
      /*
      const int value =
          observed == 0
              ? -1 // 若observed 没有被更新过,栅格值为-1，代表空闲
              // observed 被更新过，则栅格值在[0,100]，表示占用
              : ::cartographer::common::RoundToInt((1. - color / 255.) * 100.);

      // 栅格地图添加数据：-1 =< x =< 100
      CHECK_LE(-1, value);
      CHECK_GE(100, value);
      */

      /* 
      notice: 生成ROS兼容的栅格地图

       像素值65-100的设置占用值为100,表示占用,代表障碍物
       像素值0-19.6的设置占用值为0,表示空闲,代表可通过区域
       像素值在中间的值保持不变,灰色未知

      */

      
      // 适配amcl和costmap
      int value = -1;// -1表示位未知 19.6 =< x =< 65 
      if (observed != 0)
      {
        // value_temp = [0～100]
        int value_temp = ::cartographer::common::RoundToInt((1. - color / 255.) * 100.);
        // 大于65则认为是100
        if (value_temp > 100 * 0.65)
            value_temp = 100;// 占用
        // 小于19.6则认为是0
        else if (value_temp < 100 * 0.196)
            value_temp = 0;// 空闲
        
        value = value_temp;
      }


      
      occupancy_grid->data.push_back(value);
    }
  }

  return occupancy_grid;
}

}  // namespace cartographer_ros
