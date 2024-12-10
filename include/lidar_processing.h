#pragma once

#ifndef LIDARPROCESS_H
#define LIDARPROCESS_H

#include <common_pcl.h>
#include <ioctree.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

enum LID_TYPE { LIVOX = 1, VELODYNE = 2, OUSTER = 3, HESAI = 4 };

struct LidarParams {
  int type;
  int rate;
  double min_range;
  double max_range;
  double downsample_factor;
  std::string topic;
};

struct EIGEN_ALIGN16 livox_point {
  float x;
  float y;
  float z;
  uint8_t reflectivity;
  uint8_t tag;
  uint8_t line;
  uint32_t offset_time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 velodyne_point {
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 ouster_point {
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;
  uint16_t reflectivity;
  uint8_t ring;
  uint16_t ambient;
  uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 hesai_point {
  PCL_ADD_POINT4D;
  float intensity;
  double timestamp;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_point,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      float, intensity,
                                      intensity)(float, time, time)(uint16_t,
                                                                    ring, ring))
POINT_CLOUD_REGISTER_POINT_STRUCT(
    ouster_point,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        std::uint32_t, t, t)(std::uint16_t, reflectivity,
                             reflectivity)(std::uint8_t, ring,
                                           ring)(std::uint32_t, range, range))
POINT_CLOUD_REGISTER_POINT_STRUCT(
    hesai_point,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        double, timestamp, timestamp)(uint16_t, ring, ring))
POINT_CLOUD_REGISTER_POINT_STRUCT(
    livox_point,
    (float, x, x)(float, y, y)(float, z, z)(uint8_t, reflectivity,
                                            reflectivity)(uint8_t, tag, tag)(
        uint8_t, line, line)(uint32_t, offset_time, offset_time))

class LidarProcess {
 public:
  ~LidarProcess();
  LidarProcess(LidarParams params, rclcpp::Node::SharedPtr node);
  void ClearPointCloud();
  void GetPointCloud(EllipseLivoPointCloudPtr pc, rclcpp::Time &end_time);

  rclcpp::Time lidar_start_time_, lidar_end_time_;

 private:
  void LidarCallback(const sensor_msgs::msg::PointCloud2::UniquePtr msg_in);
  void Process(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void SetMinMaxTime(rclcpp::Time &point_time);
  void LivoxHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                    EllipseLivoPointCloudPtr new_pc);
  void VelodyneHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                       EllipseLivoPointCloudPtr new_pc);
  void OusterHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                     EllipseLivoPointCloudPtr new_pc);
  void HesaiHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                    EllipseLivoPointCloudPtr new_pc);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr lidar_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;

  EllipseLivoPointCloudPtr ellipselivo_pc_;
  rclcpp::Time new_lidar_start_time_, new_lidar_end_time_;

  iOctree::Octree ioctree_;

  std::mutex lidar_mutex_;
  LidarParams params_;
};

#endif  // LIDARPROCESS_H
