#pragma once

#ifndef IMU_PROCESSING_H
#define IMU_PROCESSING_H

#include <common_lib.h>
#include <common_pcl.h>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <so3_math.h>
#include <use_ikfom.h>

#include <Eigen/Eigen>
#include <boost/circular_buffer.hpp>
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

#define MAX_INI_COUNT (100)
#define LASER_POINT_COV (0.001)

class ImuProcess {
 public:
  ~ImuProcess();
  ImuProcess(KfFastlioSPtr kf, int imu_freq, std::string imu_topic,
             rclcpp::Node::SharedPtr node);

  void UndistortPointCloud(FastLioPointCloudPtr pc, KfState &kf_state,
                           rclcpp::Time lidar_end_time);
  void UpdateStatesWithLidar(double &solve_H_time, KfState &kf_state);
  void GetKfState(KfState &kf_state);

  void set_gyr_cov(const V3D &gyr_cov);
  void set_acc_cov(const V3D &acc_cov);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  void set_extrinsic(const V3D &transl, const M3D &rot);

  bool imu_need_init_;
  rclcpp::Time imu_start_time_, imu_end_time_, lidar_last_time_;

 private:
  void Reset();
  void Process(const sensor_msgs::msg::Imu::SharedPtr msg);
  void InitImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void ImuCallback(const sensor_msgs::msg::Imu::UniquePtr msg_in);
  void GetTimeMatch(int &match_idx, rclcpp::Time &match_time,
                    boost::circular_buffer<ImuState> &imu_states);

  KfFastlioSPtr kf_;
  KfState kf_state_;

  std::mutex imu_mutex_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

  boost::circular_buffer<ImuState> imu_states_;

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
  int init_iter_num;
  bool b_first_frame_;
};

#endif  // IMU_PROCESSING_H
