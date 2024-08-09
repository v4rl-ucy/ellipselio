#pragma once

#ifndef COMMON_PCL_H
#define COMMON_PCL_H

#define PCL_NO_PRECOMPILE

#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <pcl/common/impl/io.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/impl/pcd_io.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

struct EIGEN_ALIGN16 PointXYZRGBINormal {
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  union {
    struct {
      PCL_ADD_UNION_RGB
      float intensity;
      float offset_time;
      float has_color;
    };
    float data_c[4];
  };
  PCL_ADD_EIGEN_MAPS_RGB
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZRGBINormal,
    (float, x, x)(float, y, y)(float, z, z)(float, normal_x, normal_x)(
        float, normal_y, normal_y)(float, normal_z, normal_z)(float, rgb, rgb)(
        float, intensity, intensity)(float, offset_time, offset_time))

typedef PointXYZRGBINormal FastLioPoint;
typedef pcl::PointCloud<FastLioPoint> FastLioPointCloud;
typedef std::vector<FastLioPoint, Eigen::aligned_allocator<FastLioPoint>>
    PointVector;

#endif  // COMMON_PCL_H