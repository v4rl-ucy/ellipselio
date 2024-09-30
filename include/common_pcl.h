#pragma once

#ifndef COMMON_PCL_H
#define COMMON_PCL_H

#define PCL_NO_PRECOMPILE

#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/octree.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <pcl/common/impl/io.hpp>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/impl/pcd_io.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/octree/impl/octree_search.hpp>

struct EIGEN_ALIGN16 PointXYZRGBI {
  PCL_ADD_POINT4D;
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
    PointXYZRGBI,
    (float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)(
        float, intensity, intensity)(float, offset_time,
                                     offset_time)(float, has_color, has_color))

typedef PointXYZRGBI FastLioPoint;
typedef pcl::PointCloud<FastLioPoint> FastLioPointCloud;
typedef pcl::PointCloud<FastLioPoint>::Ptr FastLioPointCloudPtr;
typedef std::vector<FastLioPoint, Eigen::aligned_allocator<FastLioPoint>>
    PointVector;
typedef pcl::octree::OctreePointCloudSearch<FastLioPoint> FastLioPointOctree;
typedef pcl::octree::OctreePointCloudSearch<FastLioPoint>::Ptr
    FastLioPointOctreePtr;

#endif  // COMMON_PCL_H