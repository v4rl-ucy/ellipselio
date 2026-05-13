#ifndef IMU_PROCESSING_H_
#define IMU_PROCESSING_H_

#include <math.h>
#include <pcl_conversions/pcl_conversions.h>

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

#include "cam_processing.h"
#include "common_lib.h"

inline constexpr double kGravityMetersPerSecondSquared = 9.80665;
inline constexpr double kLidarPointCovariance = 0.001;

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

  void UndistortPointCloud(EllipseLioPointCloudPtr pc, KfState* kf_state,
                           const rclcpp::Time& lidar_start_time,
                           const rclcpp::Time& lidar_end_time,
                           const CamProcessVec& cams);
  void UpdateStatesWithLidar(KfState* kf_state,
                             const rclcpp::Time& lidar_end_time,
                             double max_solve_time);
  void GetKfState(KfState* kf_state) const;
  void SyncWithLidar(rclcpp::Time* imu_start_time, rclcpp::Time* imu_end_time);

  void SetGyrCov(const V3D& gyr_cov);
  void SetAccCov(const V3D& acc_cov);
  void SetGyrBiasCov(const V3D& b_g);
  void SetAccBiasCov(const V3D& b_a);
  void SetExtrinsic(const V3D& transl, const M3D& rot);

  std::atomic<int> imu_counter_;
  bool imu_has_data_, lidar_ready_;
  rclcpp::Time imu_start_time_, imu_end_time_;
  Eigen::Matrix<double, 12, 12> q_;

 private:
  void SetKfState();
  void Process(const sensor_msgs::msg::Imu::SharedPtr msg);
  void InitImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void ImuCallback(const sensor_msgs::msg::Imu::UniquePtr msg_in);
  void GetTimeMatch(int* match_idx, const rclcpp::Time& match_time,
                    const boost::circular_buffer<ImuState>& imu_states);
  void GetMatchingImages(const rclcpp::Time& min_time,
                         const rclcpp::Time& match_time,
                         const CamProcessVec& cams,
                         const boost::circular_buffer<ImuState>& imu_states);
  void ColorisePoint(EllipseLioPoint* pt, const CamProcessVec& cams,
                     const Eigen::Isometry3d& T_world_pt,
                     const Eigen::Isometry3d& T_imu_lidar);

  IkfomSPtr kf_;
  KfState kf_state_, pub_kf_state_;

  ImuParams params_;

  std::mutex imu_mutex_;
  mutable std::mutex pub_mutex_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

  boost::circular_buffer<ImuState> imu_states_, synced_imu_states_;

  V3D mean_acc_;
  V3D mean_gyr_;
  V3D acc_noise_;
  V3D gyr_noise_;
  V3D acc_bias_;
  V3D gyr_bias_;

  int init_iter_num_;
  double last_imu_time_;
  bool b_first_frame_, imu_need_init_;
};

#endif  // IMU_PROCESSING_H_
