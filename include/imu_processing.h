#pragma once

#ifndef IMU_PROCESSING_H
#define IMU_PROCESSING_H

#include <cam_processing.h>
#include <common_lib.h>
#include <common_pcl.h>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <so3_math.h>
#include <use_ikfom.h>

#include <Eigen/Eigen>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <fstream>
#include <geometry_msgs/msg/vector3.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>

/// *************Preconfiguration

#define MAX_INI_COUNT (10)

inline const bool time_list(FastLioPoint &x, FastLioPoint &y) {
  return (x.offset_time < y.offset_time);
};

/// *************IMU Process and undistortion
class ImuProcess {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Reset();
  // void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr
  // &lastimu);
  void Reset(double start_timestamp,
             const sensor_msgs::msg::Imu::ConstSharedPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4, 4) & T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  Eigen::Matrix<double, 12, 12> Q;
  void Process(const MeasureGroup &meas,
               esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
               FastLioPointCloud::Ptr pcl_un_, CamProcessVec &p_cams);

  ofstream fout_imu;
  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  double first_lidar_time;

 private:
  void IMU_init(const MeasureGroup &meas,
                esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas,
                    esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                    FastLioPointCloud &pcl_in_out, CamProcessVec &p_cams);

  FastLioPointCloud::Ptr cur_pcl_un_;
  // sensor_msgs::ImuConstPtr last_imu_;
  sensor_msgs::msg::Imu::ConstSharedPtr last_imu_;
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> v_imu_;
  vector<Pose6D> IMUpose;
  vector<M3D> v_rot_pcl_;
  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double start_timestamp_;
  double last_lidar_end_time_;
  int init_iter_num = 1;
  bool b_first_frame_ = true;
  bool imu_need_init_ = true;
};

#endif  // IMU_PROCESSING_H
