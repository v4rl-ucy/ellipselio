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

#define MAX_INI_COUNT (100)

/// *************IMU Process and undistortion
class ImuProcess {
 public:
  ~ImuProcess();
  ImuProcess(KfFastlioSPtr kf, StateTimeSPtr kf_state, int imu_freq,
             std::string imu_topic, rclcpp::Node::SharedPtr node);

  void set_gyr_cov(const V3D &gyr_cov);
  void set_acc_cov(const V3D &acc_cov);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  void set_extrinsic(const V3D &transl, const M3D &rot);

 private:
  void Reset();
  void Process(const sensor_msgs::msg::Imu::SharedPtr msg);
  void IMU_init(const sensor_msgs::msg::Imu::SharedPtr msg);
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void UndistortPcl(const MeasureGroup &meas,
                    esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                    FastLioPointCloud &pcl_in_out, CamProcessVec &p_cams);

  KfFastlioSPtr kf_;
  StateTimeSPtr kf_state_;

  std::mutex imu_mutex_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Time imu_start_time_, imu_end_time_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

  boost::circular_buffer<Pose6D> imu_poses_;
  boost::circular_buffer<state_time> kf_states_;
  boost::circular_buffer<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer_;

  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;

  Eigen::Matrix<double, 12, 12> Q;

  int imu_freq_;
  int init_iter_num = 1;
  bool b_first_frame_ = true;
  bool imu_need_init_ = true;
};

#endif  // IMU_PROCESSING_H
