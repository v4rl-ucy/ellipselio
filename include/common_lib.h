#pragma once

#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <common_pcl.h>
#include <so3_math.h>
#include <use_ikfom.h>

#include <Eigen/Eigen>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

using namespace std;

#define MIN_NEIGHBOURS (6)
#define MAX_NEIGHBOURS (60)
#define MIN_BIN_SIZE (1.0)
#define MIN_BIN_RESOLUTION (0.01)
#define MIN_MAP_RESOLUTION (0.1)
#define MIN_SEARCH_RADIUS (1.0)
#define MAX_SCAN_POINTS (200000)
#define MAX_PROC_POINTS (40000)
#define MAX_MAP_POINTS (10000000)

#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define QUAT_FROM_ARRAY(v) v[3], v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]

typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3d M3D;
typedef Eigen::Vector3f V3F;
typedef Eigen::Matrix3f M3F;

static M3D Eye3d(M3D::Identity());
static M3F Eye3f(M3F::Identity());
static V3D Zero3d(0, 0, 0);
static V3F Zero3f(0, 0, 0);

struct KfState {
  rclcpp::Time time = rclcpp::Time(0, 0, RCL_ROS_TIME);
  state_ikfom state;
  Ikfom::cov cov;
};

typedef std::shared_ptr<KfState> KfStateSPtr;

struct ImuState {
  KfState state;
  V3D acc;
  V3D gyr;
  V3D acc_avr;
  V3D gyr_avr;
};

#endif