#include <common_pcl.h>
#include <imu_processing.h>
#include <ioctree.h>
#include <lidar_processing.h>
#include <math.h>
#include <omp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <project_ellipse.h>
#include <so3_math.h>
#include <tf2_ros/transform_broadcaster.h>
#include <unistd.h>

#include <Eigen/Core>
#include <chrono>
#include <csignal>
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#define INIT_TIME (0.1)
#define MAXN (720000)
#define PUBFRAME_PERIOD (20)
#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]

namespace fastlio {

class LaserMappingNode : public rclcpp::Node {
 public:
  LaserMappingNode(const rclcpp::NodeOptions &options);
  ~LaserMappingNode();

 private:
  void pointLidarToWorld_ikfom(FastLioPoint const *const pi,
                               FastLioPoint *const po, state_ikfom &s);
  void pointLidarToIMU_ikfom(FastLioPoint const *const pi,
                             FastLioPoint *const po, state_ikfom &s);
  void pointLidarToWorld(FastLioPoint const *const pi, FastLioPoint *const po);
  template <typename T>
  void pointLidarToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po);
  void RGBpointLidarToWorld(FastLioPoint const *const pi,
                            FastLioPoint *const po);
  void RGBpointLidarToIMU(FastLioPoint const *const pi, FastLioPoint *const po);

  bool sync_packages();

  bool tensor_density_expection(Eigen::Vector3f &eig_val);
  void compute_tensor_vote(int i, int j, Eigen::Matrix3f &A_j, bool first_pass);
  void compute_tensor_eigen(int i, Eigen::Matrix3f &tensor, bool first_pass);
  void tensor_vote_pass_1(int old_map_size, std::vector<int> &added_idxs,
                          std::vector<int> &updated_idxs);
  void tensor_vote_pass_2(std::vector<int> &added_idxs,
                          std::vector<int> &updated_idxs);
  void compute_geometric_primitive(int map_i, int sali_idx,
                                   Eigen::Vector3f &p_world,
                                   Eigen::Vector3f &norm_vec);

  void publish_map();
  void publish_markers();
  void publish_odometry();
  void publish_frame_world();

  void tensor_registration(state_ikfom &s,
                           esekfom::dyn_share_datastruct<double> &ekfom_data);

  void timer_callback();
  void init_cam_process();
  void map_incremental(bool init_map);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pubLaserCloudFull_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubMarker_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;
  rclcpp::TimerBase::SharedPtr loop_timer_;
  rclcpp::TimerBase::SharedPtr pub_odom_timer_;
  rclcpp::TimerBase::SharedPtr pub_path_timer_;
  rclcpp::TimerBase::SharedPtr pub_scan_timer_;
  rclcpp::TimerBase::SharedPtr pub_map_timer_;
  rclcpp::TimerBase::SharedPtr pub_marker_timer_;
  rclcpp::CallbackGroup::SharedPtr loop_callback_group_;
  rclcpp::CallbackGroup::SharedPtr pub_callback_group_;

  double deltaT, deltaR,
      aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0,
      aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0,
      max_time_consu = 0, max_time_icp = 0, max_time_match = 0,
      max_time_incre = 0, max_time_solve = 0, max_time_const_H_time = 0;
  double max_imu_time = 0, max_downsample_time = 0, max_init_kdtree_time = 0,
         max_state_update_time = 0, max_kdtree_update_time = 0,
         max_total_time = 0;
  double epsi[23] = {0.001};

  /*** Time Log Variables ***/
  double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0,
         kdtree_delete_time = 0.0;
  double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN],
      s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN],
      s_plot10[MAXN], s_plot11[MAXN];
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;
  double imu_time = 0, downsample_time = 0, init_kdtree_time = 0,
         state_update_time = 0, kdtree_update_time = 0, total_time = 0;
  int kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0,
      kdtree_delete_counter = 0, pub_map_n_secs = 0;
  bool runtime_pos_log = true, pcd_save_en = false, time_sync_en = false,
       extrinsic_est_en = true, path_en = true;

  string lid_topic, imu_topic;

  double res_mean_last = 0.05, total_residual = 0.0;
  double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
  double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
  double filter_size_corner_min = 0, filter_size_surf_min = 0,
         filter_size_map_min = 0, fov_deg = 0;
  double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0,
         lidar_end_time = 0, first_lidar_time = 0.0, last_publish_time = 0.0;
  int effct_feat_num = 0, color_feat_num = 0, time_log_counter = 0,
      scan_count = 0;
  int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0,
      laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
  int cam_frame_rate = 20;

  bool lidar_pushed, flg_first_scan = true, cam_init = false, flg_exit = false,
                     flg_EKF_inited;
  bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
  bool is_first_lidar = true;

  int map_counter = 0, marker_start_idx = 0;
  float tensor_sigma = 0, tensor_radius = 0, tensor_d1 = 0, tensor_d2 = 0,
        tensor_d3 = 0;

  int map_bucket_size;
  double map_search_radius;

  int frame_num = 0;
  int lidar_type = 0, scan_rate = 10;
  double blind = 0.01;

  bool initialized = false;

  Eigen::Vector3f mean_sali;

  vector<Eigen::Matrix3f> tensors_p1;
  vector<Eigen::Matrix3f> tensors_p2;
  vector<Eigen::Matrix3f> eigenvectors;
  vector<Eigen::Vector3f> eigenvalues;
  vector<Eigen::Vector3f> salivalues;

  std::vector<int> new_neighbours_map_idx;
  std::vector<std::vector<int>> new_neighbours;
  std::vector<std::atomic<int>> new_neighbours_size;

  vector<int> update_cnt;
  vector<int> update_idx;
  vector<int> updated_pt;
  vector<int> saliency_idxs;
  vector<vector<bool>> filters;
  vector<vector<int>> neighbours;

  vector<string> cam_topics;
  vector<double> cam_intrinsics;
  vector<double> T_cam_lidars;
  vector<double> R_cam_lidars;

  vector<double> extrinT;
  vector<double> extrinR;

  FastLioPointCloudPtr map_cloud;
  FastLioPointCloudPtr feats_undistort;
  FastLioPointCloudPtr feats_down_body;
  FastLioPointCloudPtr feats_down_world;

  iOctree::Octree ioctree;

  V3D Lidar_T_wrt_IMU;
  M3D Lidar_R_wrt_IMU;

  KfFastlioSPtr kf_;
  KfState kf_state_;

  std::shared_ptr<ImuProcess> imu_process;
  std::shared_ptr<LidarProcess> lid_process;
  //  CamProcessVec p_cams;
};
}  // namespace fastlio