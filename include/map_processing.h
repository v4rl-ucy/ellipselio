/**
 * @file map_processing.h
 * @brief Main mapping and localization node for LiDAR-inertial odometry.
 * @details Orchestrates sensor processing pipelines, implements tensor voting
 * algorithm for ellipsoid representation, performs EKF state updates, and
 * publishes odometry/map.
 */

#ifndef MAP_PROCESSING_H_
#define MAP_PROCESSING_H_

#include <math.h>
#include <omp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sys/times.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ellipselio/msg/ellipse_lio_analytics.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <limits>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "common_lib.h"
#include "imu_processing.h"
#include "lidar_processing.h"

namespace ellipselio {

/**
 * @class MappingNode
 * @brief Main ROS 2 node implementing LiDAR-inertial odometry with tensor
 * voting.
 * @details Orchestrates multi-sensor fusion, implements tensor voting algorithm
 * for ellipsoid-based map representation, performs EKF state estimation, and
 *          publishes odometry, map, and analytics information.
 */
class MappingNode : public rclcpp::Node {
 public:
  /**
   * @brief Construct mapping node with ROS 2 options.
   * @param options ROS 2 node options (namespace, parameters, etc.)
   */
  MappingNode(const rclcpp::NodeOptions& options);

  /**
   * @brief Destructor.
   */
  ~MappingNode();

 private:
  /**
   * @brief Synchronize IMU, LiDAR, and camera measurements.
   * @return True if synchronization successful and new measurements available
   * @details Performs time-based alignment of multiple sensor streams.
   */
  bool SyncPackages();

  /**
   * @brief First pass of tensor voting algorithm.
   * @param old_map_size Number of points in map before update
   * @param added_idxs Indices of newly added points
   * @param updated_idxs Indices of updated points
   * @details Vote propagation and normal estimation for added points.
   */
  void TensorVotePass1(int old_map_size, std::vector<int>& added_idxs,
                       std::vector<int>& updated_idxs);

  /**
   * @brief Second pass of tensor voting algorithm.
   * @param added_idxs Indices of newly added points
   * @param updated_idxs Indices of updated points
   * @details Vote aggregation and eigenvalue decomposition.
   */
  void TensorVotePass2(std::vector<int>& added_idxs,
                       std::vector<int>& updated_idxs);

  /**
   * @brief Compute tensor vote contribution from two points.
   * @param i Index of voting point
   * @param j Index of receiving point
   * @param A_j Output: accumulated tensor at point j
   * @param first_pass Whether this is first or second voting pass
   */
  void ComputeTensorVote(int i, int j, M3F* A_j, bool first_pass);

  /**
   * @brief Compute eigendecomposition of tensor.
   * @param i Index of point to update
   * @param tensor In/out: normalized tensor for eigenanalysis
   * @param first_pass Whether this is first or second pass
   */
  void ComputeTensorEigen(int i, M3F* tensor, bool first_pass);

  /**
   * @brief Split map into multiple point clouds.
   * @param input Input point cloud to split
   * @param clouds Output: vector of split point clouds
   * @param n Number of partitions
   */
  void SplitMap(const sensor_msgs::msg::PointCloud2& input,
                std::vector<sensor_msgs::msg::PointCloud2>& clouds, size_t n);

  /// @brief Publish current map as point cloud
  void PublishMap();
  /// @brief Publish latest LiDAR scan with visualization markers
  void PublishScan();
  /// @brief Publish normal/curvature/saliency as visualization markers
  void PublishMarkers();
  /// @brief Publish LiDAR-based odometry estimate
  void PublishLidarOdometry();
  /// @brief Publish IMU-integrated odometry estimate
  void PublishImuOdometry();

  /**
   * @brief Perform point-to-ellipsoid registration using EKF.
   * @param s In/out: state vector to estimate
   * @param ekfom_data In/out: EKF measurement and covariance data
   * @details Implements tensor-voting based point-to-ellipsoid distance
   * calculation for EKF measurement updates.
   */
  void TensorRegistration(state_ikfom& s,
                          esekfom::dyn_share_datastruct<double>& ekfom_data);

  /// @brief Main loop timer callback for odometry estimation
  void TimerCallback();
  /// @brief Initialize camera processors from node parameters
  void InitCamProcess();
  /// @brief Incrementally update map with new measurements
  void MapIncremental();

  /// @brief Compute and publish memory usage statistics
  void ComputeRamUsage();
  /// @brief Compute and publish CPU time statistics
  void ComputeCpuUsage();

  /// @brief Synchronize raw LiDAR point cloud with IMU state
  void SyncRawCloudWithImu();

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_mark_;
  rclcpp::Publisher<ellipselio::msg::EllipseLioAnalytics>::SharedPtr
      pub_analytics_;

  rclcpp::TimerBase::SharedPtr loop_timer_;
  rclcpp::TimerBase::SharedPtr pub_odom_lid_timer_;
  rclcpp::TimerBase::SharedPtr pub_odom_imu_timer_;
  rclcpp::TimerBase::SharedPtr pub_map_timer_;
  rclcpp::CallbackGroup::SharedPtr pub_map_callback_group_;
  rclcpp::CallbackGroup::SharedPtr pub_odom_lid_callback_group_;
  rclcpp::CallbackGroup::SharedPtr pub_odom_imu_callback_group_;
  rclcpp::CallbackGroup::SharedPtr loop_callback_group_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;

  double last_sync_time_ = 0, max_imu_time_ = 0, max_state_time_ = 0,
         max_map_time_ = 0, max_total_time_ = 0, mean_imu_time_ = 0,
         mean_state_time_ = 0, mean_map_time_ = 0, mean_total_time_ = 0;

  std::string node_namespace_;

  int pub_map_n_secs_;
  int vel_pose_counter_ = 0;
  int map_counter_ = 0, old_map_size_ = 0, new_map_size_ = 0,
      last_map_size_ = 0;

  double start_time_;
  bool initialized_ = false;

  double map_resolution_, map_search_rad_;
  int start_bin_, mean_bin_;

  int num_processors_;
  clock_t last_cpu_, last_sys_cpu_, last_user_cpu_;

  int start_bin_cnt_ = 0;
  int ekfom_vert_cnt_ = 0;
  int ekfom_obs_cnt_ = 0;
  int ekfom_iter_cnt_ = 0;
  int ekfom_upd_cnt_ = 0;

  Eigen::ArrayXf n_res_;
  Eigen::ArrayXi n_means_;
  Eigen::ArrayXXi n_cnts_;
  Eigen::ArrayXXi n_bins_;

  Eigen::ArrayXd ekfom_data_w_;
  Eigen::ArrayXd ekfom_data_sb_;
  Eigen::ArrayXd ekfom_data_om_;
  Eigen::ArrayXd ekfom_data_oe_;
  Eigen::ArrayXXd ekfom_data_ot_;
  Eigen::ArrayXXd ekfom_data_or_;
  Eigen::VectorXd ekfom_data_h_;
  Eigen::ArrayXd ekfom_data_w_x_;
  Eigen::MatrixXd ekfom_data_h_x_;
  Eigen::MatrixXd ekfom_data_h_x_r_;
  Eigen::ArrayXd ekfom_data_h_v_;
  Eigen::MatrixXd ekfom_data_h_x_v_;

  std::vector<float> traj_dist_;
  std::vector<Eigen::Vector3f> vel_poses_;
  std::vector<Eigen::Vector3f> poses_;
  std::vector<Eigen::Quaternionf> rotes_;

  std::vector<M3F> tensors_p1_;
  std::vector<M3F> tensors_p2_;
  std::vector<M3F> eigenvectors_;
  std::vector<V3F> eigenvalues_;
  std::vector<V3F> salivalues_;

  Eigen::ArrayXi raw_cloud_bins_;
  Eigen::ArrayXi scan_cloud_bins_;
  Eigen::ArrayXi buffer_cloud_bins_;
  std::vector<std::atomic<int>> scan_bin_sizes_;
  std::vector<std::atomic<int>> filter_bin_sizes_;

  std::vector<int> new_neighbours_map_idx_;
  std::vector<std::atomic<int>> updated_pt_;
  std::vector<std::vector<int>> new_neighbours_;
  std::vector<std::atomic<int>> new_neighbours_size_;

  std::vector<int> update_idx_;
  std::vector<int> saliency_idxs_;
  std::vector<std::vector<int>> neighbours_;
  std::vector<Eigen::Vector2i> filters_;

  int num_cams_;
  std::string cam_transport_;
  std::vector<std::string> cam_topics_;
  std::vector<double> t_cam_lidars_;
  std::vector<double> r_cam_lidars_;
  std::vector<double> cam_intrinsics_;
  std::vector<long int> cam_frame_rates_;

  std::vector<double> t_imu_lidar_;
  std::vector<double> r_imu_lidar_;

  EllipseLioPointCloudPtr map_cloud_;
  EllipseLioPointCloudPtr raw_cloud_;
  EllipseLioPointCloudPtr scan_cloud_;
  EllipseLioPointCloudPtr scan_cloud_grav_;
  EllipseLioPointCloudPtr filter_cloud_;
  EllipseLioPointCloudPtr buffer_cloud_;
  EllipseLioPointCloudPtr scan_cloud_pub_;

  iOctree::Octree ioctree_;

  ImuParams imu_params_;
  LidarParams lidar_params_;

  IkfomSPtr kf_;
  KfState kf_state_, kf_state_pub_;

  std::mutex map_mutex_;
  std::mutex odom_mutex_;

  ellipselio::msg::EllipseLioAnalytics analytics_msg_;
  ellipselio::msg::EllipseLioAnalytics analytics_msg_pub_;

  rclcpp::Time last_opt_pub_time, last_imu_pub_time, last_imu_time_;
  rclcpp::Time imu_start_time_, imu_end_time_;
  rclcpp::Time raw_start_time_, raw_end_time_;
  rclcpp::Time scan_start_time_, scan_end_time_;
  rclcpp::Time buffer_start_time_, buffer_end_time_;

  double imu_time_offset_ = 0, lid_time_offset_ = 0;

  CamProcessVec cams_process_;
  std::shared_ptr<ImuProcess> imu_process_;
  std::shared_ptr<LidarProcess> lid_process_;
};
}  // namespace ellipselio

#endif  // MAP_PROCESSING_H_