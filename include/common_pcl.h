/**
 * @file common_pcl.h
 * @brief Custom PCL point types for multi-vendor LiDAR support.
 * @details Defines point cloud point types for Livox, Velodyne, Ouster, Hesai,
 *          Gazebo sensors, plus the unified EllipseLioPoint type with metadata.
 */

#ifndef COMMON_PCL_H_
#define COMMON_PCL_H_

#define PCL_NO_PRECOMPILE

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/impl/centroid.hpp>
#include <pcl/common/impl/common.hpp>
#include <pcl/common/impl/transforms.hpp>
#include <pcl/impl/point_types.hpp>

/**
 * @struct PointXYZNRGBIT
 * @brief EllipseLio unified point type with geometry, color, intensity, and
 * metadata.
 * @details Combines position (x,y,z), RGBA color, intensity, timestamp,
 *          bin index, scan index, and primitive type for the ellipsoid
 * representation. Aligned to 16 bytes for SIMD optimization.
 */
struct EIGEN_ALIGN16 PointXYZNRGBIT {
  /// @brief Homogeneous coordinate (X, Y, Z, unused)
  PCL_ADD_POINT4D;
  /// @brief Spatial metadata
  union {
    struct {
      /// @brief Range bin index for octree organization
      uint32_t bin_idx;
      /// @brief Scan line index in point cloud
      uint32_t scan_idx;
      /// @brief Flag indicating whether RGBA color is valid
      uint32_t has_rgb;
      /// @brief Primitive type (0=point, 1=line, 2=plane, 3=ellipsoid)
      uint32_t prim_type;
    };
    /// @brief Padding array for data alignment
    float data_n[4];
  };
  /// @brief Color and intensity data
  union {
    struct {
      /// @brief RGBA color (OpenCV convention: BGR packed in 32-bit int)
      PCL_ADD_UNION_RGB
      /// @brief Intensity value (0-255 or reflectance percentage)
      float intensity;
      /// @brief Timestamp seconds component
      uint32_t time_secs;
      /// @brief Timestamp nanoseconds component
      uint32_t time_nsecs;
    };
    /// @brief Padding array for data alignment
    float data_c[4];
  };
  /// @brief SIMD-aligned color access
  PCL_ADD_EIGEN_MAPS_RGB
  /// @brief Enable Eigen aligned memory operations
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZNRGBIT,
    (float, x, x)(float, y, y)(float, z, z)(uint32_t, bin_idx, bin_idx)(
        uint32_t, scan_idx, scan_idx)(uint32_t, has_rgb, has_rgb)(
        uint32_t, prim_type, prim_type)(float, rgb, rgb)(float, intensity,
                                                         intensity)(
        uint32_t, time_secs, time_secs)(uint32_t, time_nsecs, time_nsecs))

/// @brief Alias for EllipseLio unified point type
using EllipseLioPoint = PointXYZNRGBIT;
/// @brief Point cloud container for EllipseLio points
using EllipseLioPointCloud = pcl::PointCloud<EllipseLioPoint>;
/// @brief Shared pointer to EllipseLio point cloud
using EllipseLioPointCloudPtr = pcl::PointCloud<EllipseLioPoint>::Ptr;

/**
 * @struct LivoxPoint
 * @brief Livox Mid-360 / Mid-70 point type.
 * @details Native point format from Livox sensors including timestamp,
 *          intensity, tag, and scan line information.
 */
struct EIGEN_ALIGN16 LivoxPoint {
  /// @brief X coordinate in sensor frame [m]
  float x;
  /// @brief Y coordinate in sensor frame [m]
  float y;
  /// @brief Z coordinate in sensor frame [m]
  float z;
  /// @brief Reflectance intensity (0-255)
  float intensity;
  /// @brief Point tag (user-defined metadata)
  uint8_t tag;
  /// @brief Scan line number
  uint8_t line;
  /// @brief Point timestamp relative to frame start [s]
  double timestamp;
  /// @brief Enable Eigen aligned memory operations
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @struct VelodynePoint
 * @brief Velodyne VLP-16 / VLP-32 / HDL-64 point type.
 * @details Point format from Velodyne rotating LiDAR scanners with
 *          per-point timestamp offset and ring/laser ID.
 */
struct EIGEN_ALIGN16 VelodynePoint {
  /// @brief Homogeneous coordinate (X, Y, Z, unused)
  PCL_ADD_POINT4D;
  /// @brief Reflectance intensity (0-255 or 0-65535 for 16-bit sensors)
  float intensity;
  /// @brief Time offset from scan start relative to measurement time [s]
  float time;
  /// @brief Laser/ring ID (0 to number of lines - 1)
  uint16_t ring;
  /// @brief Enable Eigen aligned memory operations
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @struct OusterPoint
 * @brief Ouster OS1 / OS2 (64/128 line) point type.
 * @details Point format from Ouster solid-state LiDAR with extended
 *          metadata (reflectivity, ambient, range).
 */
struct EIGEN_ALIGN16 OusterPoint {
  /// @brief Homogeneous coordinate (X, Y, Z, unused)
  PCL_ADD_POINT4D;
  /// @brief Signal intensity at return (0-255)
  float intensity;
  /// @brief Measurement timestamp [nanoseconds]
  uint32_t t;
  /// @brief Reflectivity estimate (0-255)
  uint16_t reflectivity;
  /// @brief Ambient light level
  uint16_t ambient;
  /// @brief Raw range measurement [millimeters]
  uint32_t range;
  /// @brief Enable Eigen aligned memory operations
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @struct HesaiPoint
 * @brief Hesai QT / XT / QT64 point type.
 * @details Point format from Hesai lidar scanners with per-point
 *          timestamp and ring/laser information.
 */
struct EIGEN_ALIGN16 HesaiPoint {
  /// @brief Homogeneous coordinate (X, Y, Z, unused)
  PCL_ADD_POINT4D;
  /// @brief Reflectance intensity (0-255 or higher for 16-bit sensors)
  float intensity;
  /// @brief Point timestamp [seconds]
  double timestamp;
  /// @brief Laser/ring ID (0 to number of lines - 1)
  uint16_t ring;
  /// @brief Enable Eigen aligned memory operations
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @struct GazeboPoint
 * @brief Gazebo simulator lidar point type.
 * @details Point format used by Gazebo's ray plugin for simulation,
 *          compatible with typical rotating LiDAR sensor models.
 */
struct EIGEN_ALIGN16 GazeboPoint {
  /// @brief Homogeneous coordinate (X, Y, Z, unused)
  PCL_ADD_POINT4D;
  /// @brief Simulated intensity value (0-255)
  float intensity;
  /// @brief Simulated laser/ring ID
  uint16_t ring;
  /// @brief Enable Eigen aligned memory operations
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

#endif  // COMMON_PCL_H_