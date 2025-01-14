#pragma once

#ifndef COMMON_PCL_H
#define COMMON_PCL_H

#define PCL_NO_PRECOMPILE

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <pcl/impl/point_types.hpp>

struct EIGEN_ALIGN16 PointXYZNRGBIT {
  PCL_ADD_POINT4D;
  union {
    struct {
      uint32_t bin_idx;
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
        float, rgb, rgb)(float, intensity, intensity)(
        uint32_t, time_secs, time_secs)(uint32_t, time_nsecs, time_nsecs))

typedef PointXYZNRGBIT EllipseLivoPoint;
typedef pcl::PointCloud<EllipseLivoPoint> EllipseLivoPointCloud;
typedef pcl::PointCloud<EllipseLivoPoint>::Ptr EllipseLivoPointCloudPtr;

struct EIGEN_ALIGN16 LivoxPoint {
  float x;
  float y;
  float z;
  uint8_t reflectivity;
  uint8_t tag;
  uint8_t line;
  uint32_t offset_time;
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
  uint8_t ring;
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

POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePoint,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      float, intensity,
                                      intensity)(float, time, time)(uint16_t,
                                                                    ring, ring))
POINT_CLOUD_REGISTER_POINT_STRUCT(
    OusterPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        std::uint32_t, t, t)(std::uint16_t, reflectivity,
                             reflectivity)(std::uint8_t, ring,
                                           ring)(std::uint32_t, range, range))
POINT_CLOUD_REGISTER_POINT_STRUCT(
    HesaiPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        double, timestamp, timestamp)(uint16_t, ring, ring))
POINT_CLOUD_REGISTER_POINT_STRUCT(
    LivoxPoint,
    (float, x, x)(float, y, y)(float, z, z)(uint8_t, reflectivity,
                                            reflectivity)(uint8_t, tag, tag)(
        uint8_t, line, line)(uint32_t, offset_time, offset_time))

#endif  // COMMON_PCL_H