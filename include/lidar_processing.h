#pragma once

#ifndef LIDARPROCESS_H
#define LIDARPROCESS_H

#include <common_pcl.h>
#include <ioctree.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std;

enum LID_TYPE { LIVOX = 1, VELODYNE = 2, OUSTER = 3, HESAI = 4 };

enum TIME_UNIT { SEC = 0, MS = 1, US = 2, NS = 3 };

struct EIGEN_ALIGN16 livox_point {
  float x;              /**< X axis, Unit:m */
  float y;              /**< Y axis, Unit:m */
  float z;              /**< Z axis, Unit:m */
  uint8_t reflectivity; /**< Reflectivity   */
  uint8_t tag;          /**< Livox point tag   */
  uint8_t line;         /**< Laser line id     */
  uint32_t offset_time; /**< Time offset, Unit:ns */
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

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity,intensity)
                                  (float, time, time)
                                  (uint16_t, ring, ring)
)
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint32_t, t, t)
                                  (std::uint16_t, reflectivity, reflectivity)
                                  (std::uint8_t, ring, ring)
                                  (std::uint32_t, range, range)
)
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(hesai_point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (double, timestamp,timestamp)
                                  (uint16_t, ring, ring) 
)
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(livox_point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (uint8_t, reflectivity, reflectivity)
                                  (uint8_t, tag, tag)
                                  (uint8_t, line, line)
                                  (uint32_t, offset_time, offset_time)
)

class LidarProcess
{
  public:

  LidarProcess();
  ~LidarProcess();
  
  void process(const sensor_msgs::msg::PointCloud2::UniquePtr &msg, FastLioPointCloud::Ptr &pcl_out);

  FastLioPointCloud fastlio_pc_;

private:
  void livox_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
  void velodyne_handler( const sensor_msgs::msg::PointCloud2::UniquePtr &msg );
  void ouster_handler( const sensor_msgs::msg::PointCloud2::UniquePtr &msg );
  void hesai_handler( const sensor_msgs::msg::PointCloud2::UniquePtr &msg );

  iOctree::Octree ioctree_;

  int lidar_type_;
  float min_range_, max_range_, mean_range_, time_unit_scale_, scan_min_extent_;
};

#endif  // LIDARPROCESS_H
