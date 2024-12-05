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
      float normal_x;
      float normal_y;
      float normal_z;
      float curvature;
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
  PCL_ADD_EIGEN_MAPS_NORMAL4D
  PCL_ADD_EIGEN_MAPS_RGB
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZNRGBIT,
    (float, x, x)(float, y, y)(float, z, z)(float, normal_x, normal_x)(
        float, normal_y, normal_y)(float, normal_z, normal_z)(
        float, curvature, curvature)(float, rgb, rgb)(float, intensity,
                                                      intensity)(
        uint32_t, time_secs, time_secs)(uint32_t, time_nsecs, time_nsecs))

typedef PointXYZNRGBIT FastLioPoint;
typedef pcl::PointCloud<FastLioPoint> FastLioPointCloud;
typedef pcl::PointCloud<FastLioPoint>::Ptr FastLioPointCloudPtr;

#endif  // COMMON_PCL_H