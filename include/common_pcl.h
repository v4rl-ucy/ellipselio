#pragma once

#ifndef COMMON_PCL_H
#define COMMON_PCL_H

#define PCL_NO_PRECOMPILE

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/common/impl/common.hpp>
#include <pcl/common/impl/transforms.hpp>
#include <pcl/impl/point_types.hpp>

struct EIGEN_ALIGN16 PointXYZNRGBIT {
  PCL_ADD_POINT4D;
  union {
    struct {
      uint32_t bin_idx;
      uint32_t scan_idx;
      uint32_t has_rgb;
      uint32_t prim_type;
    };
    float data_n[4];
  };
  union {
    struct {
      PCL_ADD_UNION_RGB
      float intensity;
      uint32_t time_secs;
      uint32_t time_nsecs;
    };
    float data_c[4];
  };
  PCL_ADD_EIGEN_MAPS_RGB
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZNRGBIT,
    (float, x, x)(float, y, y)(float, z, z)(uint32_t, bin_idx, bin_idx)(
        uint32_t, has_rgb,
        has_rgb)(uint32_t, prim_type,
                 prim_type)(float, rgb, rgb)(float, intensity, intensity)(
        uint32_t, time_secs, time_secs)(uint32_t, time_nsecs, time_nsecs))

typedef PointXYZNRGBIT EllipseLioPoint;
typedef pcl::PointCloud<EllipseLioPoint> EllipseLioPointCloud;
typedef pcl::PointCloud<EllipseLioPoint>::Ptr EllipseLioPointCloudPtr;

struct EIGEN_ALIGN16 LivoxPoint {
  float x;
  float y;
  float z;
  float intensity;
  uint8_t tag;
  uint8_t line;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 VelodynePoint {
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 OusterPoint {
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;
  uint16_t reflectivity;
  uint16_t ambient;
  uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 HesaiPoint {
  PCL_ADD_POINT4D;
  float intensity;
  double timestamp;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 GazeboPoint {
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    LivoxPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        uint8_t, tag, tag)(uint8_t, line, line)(double, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePoint,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      float, intensity,
                                      intensity)(float, time, time)(uint16_t,
                                                                    ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    OusterPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        std::uint32_t, t, t)(std::uint16_t, reflectivity,
                             reflectivity)(std::uint32_t, range, range))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    HesaiPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        double, timestamp, timestamp)(uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    GazeboPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(uint16_t, ring, ring))

#endif  // COMMON_PCL_H