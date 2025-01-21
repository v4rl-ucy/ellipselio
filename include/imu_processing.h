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

#define G_m_s2 (9.80665)
#define LIDAR_PT_COV (0.001)

struct ImuParams {
  int rate;
  double gyr_noise;
  double acc_noise;
  double gyr_bias;
  double acc_bias;
  std::string topic;
  V3D t_imu_lidar;
  M3D r_imu_lidar;
};

class ImuProcess {
 public:
  ~ImuProcess();
  ImuProcess(IkfomSPtr kf, ImuParams params, rclcpp::Node::SharedPtr node);

  void UndistortPointCloud(EllipseLivoPointCloudPtr pc, KfState &kf_state,
                           rclcpp::Time &lidar_start_time,
                           rclcpp::Time &lidar_end_time, CamProcessVec &cams);
  void UpdateStatesWithLidar(KfState &kf_state, rclcpp::Time &lidar_end_time);
  void GetKfState(KfState &kf_state);

  void set_gyr_cov(const V3D &gyr_cov);
  void set_acc_cov(const V3D &acc_cov);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  void set_extrinsic(const V3D &transl, const M3D &rot);

  bool imu_has_data_;
  rclcpp::Time imu_start_time_, imu_end_time_;

 private:
  void Process(const sensor_msgs::msg::Imu::SharedPtr msg);
  void InitImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void ImuCallback(const sensor_msgs::msg::Imu::UniquePtr msg_in);
  void GetTimeMatch(int &match_idx, rclcpp::Time &match_time,
                    boost::circular_buffer<ImuState> &imu_states);
  void GetMatchingImages(rclcpp::Time &lidar_start_time, CamProcessVec &cams,
                         boost::circular_buffer<ImuState> &imu_states);
  void ColorisePoint(EllipseLivoPoint &pt, CamProcessVec &cams,
                     Eigen::Isometry3d &T_world_pt,
                     Eigen::Isometry3d &T_imu_lidar);

  IkfomSPtr kf_;
  KfState kf_state_;

  ImuParams params_;

  std::mutex imu_mutex_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

  boost::circular_buffer<ImuState> imu_states_;

  V3D mean_acc;
  V3D mean_gyr;
  V3D acc_noise;
  V3D gyr_noise;
  V3D acc_bias;
  V3D gyr_bias;

  Eigen::Matrix<double, 12, 12> Q;

  int init_iter_num;
  bool b_first_frame_, imu_need_init_;
};

#endif  // IMU_PROCESSING_H
