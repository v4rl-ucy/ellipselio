#ifndef ELLIPSE_LIO_INCLUDE_COMMON_LIB_H_
#define ELLIPSE_LIO_INCLUDE_COMMON_LIB_H_

#include <Eigen/Eigen>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "common_pcl.h"
#include "so3_math.h"
#include "use_ikfom.h"

inline constexpr int kMinNeighbours = 6;
inline constexpr int kMaxNeighbours = 60;
inline constexpr double kMinMapRes = 0.1;
inline constexpr double kMinBinRes = 0.01;
inline constexpr double kMinSearchRes = 0.1;
inline constexpr double kMaxSearchRes = 1.0;
inline constexpr int kMinProcPoints = 1000;
inline constexpr int kMaxProcPoints = 30000;
inline constexpr int kMaxScanPoints = 200000;
inline constexpr int kMaxMapPoints = 10000000;

using V3D = Eigen::Vector3d;
using M3D = Eigen::Matrix3d;
using V3F = Eigen::Vector3f;
using M3F = Eigen::Matrix3f;

template <typename ContainerT>
inline Eigen::Vector3d Vec3dFromArray(const ContainerT& values) {
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

template <typename ContainerT>
inline Eigen::Quaterniond QuaterniondFromArray(const ContainerT& values) {
  return Eigen::Quaterniond(values[3], values[0], values[1], values[2]);
}

template <typename ContainerT>
inline Eigen::Matrix3d Mat3dFromArray(const ContainerT& values) {
  Eigen::Matrix3d matrix;
  matrix << values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7],
      values[8];
  return matrix;
}

static M3D Eye3d(M3D::Identity());
static M3F Eye3f(M3F::Identity());
static V3D Zero3d(0, 0, 0);
static V3F Zero3f(0, 0, 0);

struct KfState {
  rclcpp::Time time = rclcpp::Time(0, 0, RCL_ROS_TIME);
  state_ikfom state;
  Ikfom::cov cov;
  V3D gyr;
};

using KfStateSPtr = std::shared_ptr<KfState>;

struct ImuState {
  KfState state;
  V3D acc;
  V3D gyr;
  V3D acc_avr;
  V3D gyr_avr;
};

#endif  // ELLIPSE_LIO_INCLUDE_COMMON_LIB_H_