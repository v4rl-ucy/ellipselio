#pragma once

#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <common_pcl.h>
#include <so3_math.h>
#include <use_ikfom.h>

#include <Eigen/Eigen>
#include <deque>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

using namespace std;
using namespace Eigen;

#define PI_M (3.14159265358)
#define G_m_s2 (9.81)    // Gravaty const in GuangDong/China
#define DIM_STATE (18)   // Dimension of states (Let Dim(SO(3)) = 3)
#define DIM_PROC_N (12)  // Dimension of process noise (Let Dim(SO(3)) = 3)
#define CUBE_LEN (6.0)
#define LIDAR_SP_LEN (2)
#define INIT_COV (1)
#define NUM_MATCH_POINTS (5)
#define MAX_NEIGHBOURS (25)
#define MAX_MEAS_DIM (10000)

#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
#define CONSTRAIN(v, min, max) ((v > min) ? ((v < max) ? v : max) : min)
#define ARRAY_FROM_EIGEN(mat) mat.data(), mat.data() + mat.rows() * mat.cols()
#define STD_VEC_FROM_EIGEN(mat)             \
  vector<decltype(mat)::Scalar>(mat.data(), \
                                mat.data() + mat.rows() * mat.cols())

typedef Vector3d V3D;
typedef Matrix3d M3D;
typedef Vector3f V3F;
typedef Matrix3f M3F;

#define MD(a, b) Matrix<double, (a), (b)>
#define VD(a) Matrix<double, (a), 1>
#define MF(a, b) Matrix<float, (a), (b)>
#define VF(a) Matrix<float, (a), 1>

static M3D Eye3d(M3D::Identity());
static M3F Eye3f(M3F::Identity());
static V3D Zero3d(0, 0, 0);
static V3F Zero3f(0, 0, 0);

struct MeasureGroup  // Lidar data and imu dates for the curent process
{
  MeasureGroup() {
    lidar_beg_time = 0.0;
    this->lidar.reset(new FastLioPointCloud());
  };
  double lidar_beg_time;
  double lidar_end_time;
  FastLioPointCloud::Ptr lidar;
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu;
};

struct StatesGroup {
  StatesGroup() {
    this->rot_end = M3D::Identity();
    this->pos_end = Zero3d;
    this->vel_end = Zero3d;
    this->bias_g = Zero3d;
    this->bias_a = Zero3d;
    this->gravity = Zero3d;
    this->cov = MD(DIM_STATE, DIM_STATE)::Identity() * INIT_COV;
    this->cov.block<9, 9>(9, 9) = MD(9, 9)::Identity() * 0.00001;
  };

  StatesGroup(const StatesGroup &b) {
    this->rot_end = b.rot_end;
    this->pos_end = b.pos_end;
    this->vel_end = b.vel_end;
    this->bias_g = b.bias_g;
    this->bias_a = b.bias_a;
    this->gravity = b.gravity;
    this->cov = b.cov;
  };

  StatesGroup &operator=(const StatesGroup &b) {
    this->rot_end = b.rot_end;
    this->pos_end = b.pos_end;
    this->vel_end = b.vel_end;
    this->bias_g = b.bias_g;
    this->bias_a = b.bias_a;
    this->gravity = b.gravity;
    this->cov = b.cov;
    return *this;
  };

  StatesGroup operator+(const Matrix<double, DIM_STATE, 1> &state_add) {
    StatesGroup a;
    a.rot_end =
        this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
    a.pos_end = this->pos_end + state_add.block<3, 1>(3, 0);
    a.vel_end = this->vel_end + state_add.block<3, 1>(6, 0);
    a.bias_g = this->bias_g + state_add.block<3, 1>(9, 0);
    a.bias_a = this->bias_a + state_add.block<3, 1>(12, 0);
    a.gravity = this->gravity + state_add.block<3, 1>(15, 0);
    a.cov = this->cov;
    return a;
  };

  StatesGroup &operator+=(const Matrix<double, DIM_STATE, 1> &state_add) {
    this->rot_end =
        this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
    this->pos_end += state_add.block<3, 1>(3, 0);
    this->vel_end += state_add.block<3, 1>(6, 0);
    this->bias_g += state_add.block<3, 1>(9, 0);
    this->bias_a += state_add.block<3, 1>(12, 0);
    this->gravity += state_add.block<3, 1>(15, 0);
    return *this;
  };

  Matrix<double, DIM_STATE, 1> operator-(const StatesGroup &b) {
    Matrix<double, DIM_STATE, 1> a;
    M3D rotd(b.rot_end.transpose() * this->rot_end);
    a.block<3, 1>(0, 0) = Log(rotd);
    a.block<3, 1>(3, 0) = this->pos_end - b.pos_end;
    a.block<3, 1>(6, 0) = this->vel_end - b.vel_end;
    a.block<3, 1>(9, 0) = this->bias_g - b.bias_g;
    a.block<3, 1>(12, 0) = this->bias_a - b.bias_a;
    a.block<3, 1>(15, 0) = this->gravity - b.gravity;
    return a;
  };

  void resetpose() {
    this->rot_end = M3D::Identity();
    this->pos_end = Zero3d;
    this->vel_end = Zero3d;
  }

  M3D rot_end;  // the estimated attitude (rotation matrix) at the end lidar
                // point
  V3D pos_end;  // the estimated position at the end lidar point (world frame)
  V3D vel_end;  // the estimated velocity at the end lidar point (world frame)
  V3D bias_g;   // gyroscope bias
  V3D bias_a;   // accelerator bias
  V3D gravity;  // the estimated gravity acceleration
  Matrix<double, DIM_STATE, DIM_STATE> cov;  // states covariance
};

template <typename T>
T rad2deg(T radians) {
  return radians * 180.0 / PI_M;
}

template <typename T>
T deg2rad(T degrees) {
  return degrees * PI_M / 180.0;
}

struct KfState {
  rclcpp::Time time = rclcpp::Time(0, 0, RCL_ROS_TIME);
  state_ikfom state;
  KfFastlio::cov cov;
};

typedef std::shared_ptr<KfState> KfStateSPtr;

struct ImuState {
  KfState state;
  Eigen::Vector3d acc;
  Eigen::Vector3d gyr;
  Eigen::Vector3d acc_avr;
  Eigen::Vector3d gyr_avr;
};

#endif