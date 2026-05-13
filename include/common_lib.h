/**
 * @file common_lib.h
 * @brief Common type definitions, constants, and utility functions for
 * EllipseLIO.
 * @details Provides shared type aliases for Eigen matrices/vectors, physical
 * constants, and helper functions used throughout the package.
 */

#ifndef COMMON_LIB_H_
#define COMMON_LIB_H_

#include <Eigen/Eigen>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "common_pcl.h"
#include "ikfom/use_ikfom.h"
#include "ioctree/ioctree.h"

/// @name Physical Constants
/// @{
/// @brief Minimum number of neighbors for octree operations
inline constexpr int kMinNeighbours = 6;
/// @brief Maximum number of neighbors for octree operations
inline constexpr int kMaxNeighbours = 60;
/// @brief Minimum map resolution in meters
inline constexpr double kMinMapRes = 0.1;
/// @brief Minimum bin resolution in meters
inline constexpr double kMinBinRes = 0.01;
/// @brief Minimum search radius in meters
inline constexpr double kMinSearchRes = 0.1;
/// @brief Maximum search radius in meters
inline constexpr double kMaxSearchRes = 1.0;
/// @brief Minimum number of points for processing
inline constexpr int kMinProcPoints = 1000;
/// @brief Maximum number of points for processing
inline constexpr int kMaxProcPoints = 30000;
/// @brief Maximum number of points in a single scan
inline constexpr int kMaxScanPoints = 200000;
/// @brief Maximum number of points in map
inline constexpr int kMaxMapPoints = 10000000;
/// @}

/// @name Type Aliases
/// @brief Convenient type aliases for Eigen types
/// @{
/// @brief 3D double-precision vector
using V3D = Eigen::Vector3d;
/// @brief 3x3 double-precision matrix
using M3D = Eigen::Matrix3d;
/// @brief 3D single-precision vector
using V3F = Eigen::Vector3f;
/// @brief 3x3 single-precision matrix
using M3F = Eigen::Matrix3f;
/// @}

/// @name Conversion Utilities
/// @brief Helper functions for converting between containers and Eigen types
/// @{

/**
 * @brief Convert container to 3D double-precision vector.
 * @tparam ContainerT Container type with at least 3 elements
 * @param values Input container with [x, y, z] values
 * @return Eigen::Vector3d constructed from first 3 elements
 */
template <typename ContainerT>
inline Eigen::Vector3d Vec3dFromArray(const ContainerT& values) {
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

/**
 * @brief Convert container to double-precision quaternion.
 * @tparam ContainerT Container type with at least 4 elements
 * @param values Input container with [x, y, z, w] quaternion components
 * @return Eigen::Quaterniond with scalar-last convention (w is scalar)
 */
template <typename ContainerT>
inline Eigen::Quaterniond QuaterniondFromArray(const ContainerT& values) {
  return Eigen::Quaterniond(values[3], values[0], values[1], values[2]);
}

/**
 * @brief Convert container to 3x3 double-precision matrix.
 * @tparam ContainerT Container type with at least 9 elements
 * @param values Input container with 9 matrix elements in row-major order
 * @return Eigen::Matrix3d (3x3 matrix)
 */
template <typename ContainerT>
inline Eigen::Matrix3d Mat3dFromArray(const ContainerT& values) {
  Eigen::Matrix3d matrix;
  matrix << values[0], values[1], values[2], values[3], values[4], values[5],
      values[6], values[7], values[8];
  return matrix;
}
/// @}

/// @name Global Matrix/Vector Constants
/// @{
static M3D Eye3d(M3D::Identity());  ///< 3x3 double-precision identity matrix
static M3F Eye3f(M3F::Identity());  ///< 3x3 single-precision identity matrix
static V3D Zero3d(0, 0, 0);         ///< 3D double-precision zero vector
static V3F Zero3f(0, 0, 0);         ///< 3D single-precision zero vector
/// @}

/**
 * @struct KfState
 * @brief Complete state of the Kalman filter at a given time.
 * @details Contains the IMU-centric state (position, velocity, rotation),
 *          covariance matrix, and latest gyroscope measurement.
 */
struct KfState {
  /// @brief Timestamp of this state estimate
  rclcpp::Time time = rclcpp::Time(0, 0, RCL_ROS_TIME);
  /// @brief IMU-frame state vector (position, velocity, rotation, bias)
  state_ikfom state;
  /// @brief State covariance matrix
  Ikfom::cov cov;
  /// @brief Latest gyroscope measurement
  V3D gyr;
};

/// @brief Shared pointer to KfState
using KfStateSPtr = std::shared_ptr<KfState>;

/**
 * @struct ImuState
 * @brief IMU measurement and corresponding state at a point in time.
 * @details Used to store synchronized IMU measurements with their corresponding
 *          Kalman filter state for trajectory reconstruction.
 */
struct ImuState {
  /// @brief Kalman filter state at this IMU measurement time
  KfState state;
  /// @brief Raw acceleration measurement from IMU
  V3D acc;
  /// @brief Raw gyroscope measurement from IMU
  V3D gyr;
  /// @brief Time-averaged acceleration over integration period
  V3D acc_avr;
  /// @brief Time-averaged gyroscope over integration period
  V3D gyr_avr;
};

#endif  // COMMON_LIB_H_