/**
 * @file imu_processing.h
 * @brief IMU sensor processing with EKF integration and temporal
 * synchronization.
 * @details Handles IMU measurement buffering, gravity/bias initialization,
 *          point cloud undistortion using IMU trajectory, and Kalman filter
 * state management.
 */

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

/// @brief Standard gravity acceleration (m/s²)
inline constexpr double kGravityMetersPerSecondSquared = 9.80665;
/// @brief Default point covariance for LiDAR measurements
inline constexpr double kLidarPointCovariance = 0.001;

/**
 * @struct ImuParams
 * @brief IMU sensor parameters and calibration data.
 * @details Stores IMU-specific configuration, noise models, and rigid-body
 *          transformation between IMU and LiDAR coordinate frames.
 */
struct ImuParams {
  /// @brief IMU measurement rate in Hz
  int rate;
  /// @brief Gyroscope noise standard deviation (rad/s/√Hz)
  double gyr_noise;
  /// @brief Accelerometer noise standard deviation (m/s²/√Hz)
  double acc_noise;
  /// @brief Gyroscope bias standard deviation (rad/s)
  double gyr_bias;
  /// @brief Accelerometer bias standard deviation (m/s²)
  double acc_bias;
  /// @brief ROS 2 topic name for IMU subscription
  std::string topic;
  /// @brief Translation from IMU to LiDAR frame [m]
  V3D t_imu_lidar;
  /// @brief Rotation matrix from IMU to LiDAR frame
  M3D r_imu_lidar;
};

/**
 * @class ImuProcess
 * @brief Processes IMU measurements and integrates them with EKF odometry.
 * @details Manages IMU measurement buffer, performs gravity initialization,
 *          undistorts point clouds using IMU trajectory, updates state
 * estimates, and synchronizes with camera measurements for colorization.
 */

class ImuProcess {
 public:
  /**
   * @brief Destructor.
   */
  ~ImuProcess();

  /**
   * @brief Construct an IMU processor with EKF filter and parameters.
   * @param kf Extended Kalman filter for state estimation
   * @param params IMU sensor calibration and configuration
   * @param node Shared pointer to ROS 2 node for subscription and logging
   */
  ImuProcess(IkfomSPtr kf, ImuParams params, rclcpp::Node::SharedPtr node);

  /**
   * @brief Undistort LiDAR points using IMU motion trajectory.
   * @param pc In/out: point cloud to undistort (modified in-place)
   * @param kf_state Output: Kalman filter state at lidar end time
   * @param lidar_start_time Timestamp of first LiDAR point
   * @param lidar_end_time Timestamp of last LiDAR point
   * @param cams Camera processor vector for point colorization
   * @details Removes motion distortion by applying inverse IMU trajectory
   *          to each point. Also performs time-synchronized color projection
   *          using camera measurements when available.
   */
  void UndistortPointCloud(EllipseLioPointCloudPtr pc, KfState* kf_state,
                           const rclcpp::Time& lidar_start_time,
                           const rclcpp::Time& lidar_end_time,
                           const CamProcessVec& cams);

  /**
   * @brief Update Kalman filter state with LiDAR measurement.
   * @param kf_state In/out: state estimate to update
   * @param lidar_end_time Time stamp of LiDAR measurement
   * @param max_solve_time Maximum time budget for solver in seconds
   * @details Performs EKF measurement update step to incorporate LiDAR
   *          point association constraints.
   */
  void UpdateStatesWithLidar(KfState* kf_state,
                             const rclcpp::Time& lidar_end_time,
                             double max_solve_time);

  /**
   * @brief Get current publishable Kalman filter state (thread-safe).
   * @param kf_state Output: copy of current Kalman filter state
   * @details Acquires mutex lock to safely copy state. Used for publishing
   *          odometry and trajectory information.
   */
  void GetKfState(KfState* kf_state) const;

  /**
   * @brief Synchronize IMU measurements with LiDAR frame.
   * @param imu_start_time Output: timestamp of first IMU measurement in frame
   * @param imu_end_time Output: timestamp of last IMU measurement in frame
   * @details Extracts IMU measurements that span the current LiDAR scan
   * interval and integrates them for state propagation.
   */
  void SyncWithLidar(rclcpp::Time* imu_start_time, rclcpp::Time* imu_end_time);

  /**
   * @brief Set gyroscope measurement covariance.
   * @param gyr_cov Diagonal elements of gyroscope covariance (rad²/s²)
   */
  void SetGyrCov(const V3D& gyr_cov);

  /**
   * @brief Set accelerometer measurement covariance.
   * @param acc_cov Diagonal elements of accelerometer covariance (m²/s⁴)
   */
  void SetAccCov(const V3D& acc_cov);

  /**
   * @brief Set gyroscope bias covariance.
   * @param b_g Diagonal elements of gyroscope bias covariance
   */
  void SetGyrBiasCov(const V3D& b_g);

  /**
   * @brief Set accelerometer bias covariance.
   * @param b_a Diagonal elements of accelerometer bias covariance
   */
  void SetAccBiasCov(const V3D& b_a);

  /**
   * @brief Update IMU to LiDAR extrinsic calibration.
   * @param transl Translation vector from LiDAR to IMU [m]
   * @param rot Rotation matrix from LiDAR to IMU frame
   */
  void SetExtrinsic(const V3D& transl, const M3D& rot);

  /// @brief Counter of received IMU measurements
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
