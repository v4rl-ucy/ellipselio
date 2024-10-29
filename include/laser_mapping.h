// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <Python.h>
#include <cam_processing.h>
#include <common_pcl.h>
#include <ikd_tree.h>
#include <imu_processing.h>
#include <ioctree/ioctree.h>
#include <math.h>
#include <omp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <preprocess.h>
#include <project_ellipse/project_ellipse.h>
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

#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)
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
  void dump_lio_state_to_log(FILE *fp);
  void pointLidarToWorld_ikfom(FastLioPoint const *const pi,
                               FastLioPoint *const po, state_ikfom &s);
  void pointLidarToIMU_ikfom(FastLioPoint const *const pi,
                             FastLioPoint *const po, state_ikfom &s);
  void pointLidarToWorld(FastLioPoint const *const pi, FastLioPoint *const po);
  template <typename T>
  void pointLidarToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po);
  void RGBpointLidarToWorld(FastLioPoint const *const pi,
                            FastLioPoint *const po);
  void RGBpointLidarLidarToIMU(FastLioPoint const *const pi,
                               FastLioPoint *const po);
  void points_cache_collect();
  void lasermap_fov_segment();
  void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg);
  void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg);
  void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in);
  bool sync_packages(MeasureGroup &meas, CamProcessVec &p_cams);

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

  void map_incremental(bool init_map);
  void publish_frame_world();
  void publish_frame_body();
  void publish_effect_world();
  void publish_map();
  void save_to_pcd();
  template <typename T>
  void set_posestamp(T &out);
  void publish_odometry();
  void publish_path();

  void h_share_model(state_ikfom &s,
                     esekfom::dyn_share_datastruct<double> &ekfom_data);
  void compute_eigendecomposition(
      state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);
  void tensor_registration(state_ikfom &s,
                           esekfom::dyn_share_datastruct<double> &ekfom_data);

  void init_cam_process();
  void timer_callback();
  //   void map_publish_callback();
  void map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req,
                         std_srvs::srv::Trigger::Response::SharedPtr res);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pubLaserCloudFull_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pubLaserCloudFull_body_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pubLaserCloudEffect_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr
      sub_pcl_livox_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;
  rclcpp::TimerBase::SharedPtr loop_timer_;
  rclcpp::TimerBase::SharedPtr pub_odom_timer_;
  rclcpp::TimerBase::SharedPtr pub_path_timer_;
  rclcpp::TimerBase::SharedPtr pub_scan_timer_;
  rclcpp::TimerBase::SharedPtr pub_map_timer_;
  rclcpp::CallbackGroup::SharedPtr loop_callback_group_;
  rclcpp::CallbackGroup::SharedPtr pub_callback_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;

  bool effect_pub_en = false, map_pub_en = false;
  int effect_feat_num = 0, frame_num = 0;
  double deltaT, deltaR,
      aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0,
      aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0,
      max_time_consu = 0, max_time_icp = 0, max_time_match = 0,
      max_time_incre = 0, max_time_solve = 0, max_time_const_H_time = 0;
  double max_imu_time = 0, max_downsample_time = 0, max_init_kdtree_time = 0,
         max_state_update_time = 0, max_kdtree_update_time = 0,
         max_total_time = 0;
  bool flg_EKF_converged, EKF_stop_flg = 0;
  double epsi[23] = {0.001};

  FILE *fp;
  ofstream fout_pre, fout_out, fout_dbg;

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
  bool runtime_pos_log = false, pcd_save_en = false, time_sync_en = false,
       extrinsic_est_en = true, path_en = true;
  /**************************/

  float res_last[100000] = {0.0};
  float DET_RANGE = 300.0f;
  const float MOV_THRESHOLD = 1.5f;
  double time_diff_lidar_to_imu = 0.0;

  mutex mtx_buffer;
  condition_variable sig_buffer;

  string root_dir = ROOT_DIR;
  string map_file_path, lid_topic, imu_topic;

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
  bool point_selected_surf[100000] = {0};
  bool lidar_pushed, flg_first_scan = true, cam_init = false, flg_exit = false,
                     flg_EKF_inited;
  bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
  bool is_first_lidar = true;

  int map_counter = 0;
  float tensor_sigma = 0, tensor_radius = 0, tensor_d1 = 0, tensor_d2 = 0,
        tensor_d3 = 0;

  Eigen::Vector3f mean_sali;

  vector<Eigen::Matrix3f> tensors_p1;
  vector<Eigen::Matrix3f> tensors_p2;
  vector<Eigen::Matrix3f> eigenvectors;
  vector<Eigen::Vector3f> salivalues;

  std::vector<int> new_neighbours_map_idx;
  std::vector<std::atomic<int>> new_neighbours_size;
  std::vector<std::vector<int>> new_neighbours;

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

  vector<vector<int>> pointSearchInd_surf;
  vector<BoxPointType> cub_needrm;
  vector<PointVector> Nearest_Points;
  vector<double> extrinT;
  vector<double> extrinR;
  deque<double> time_buffer;
  deque<FastLioPointCloud::Ptr> lidar_buffer;
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

  FastLioPointCloud::Ptr map_cloud;
  FastLioPointCloud::Ptr featsFromMap;
  FastLioPointCloud::Ptr feats_undistort;
  FastLioPointCloud::Ptr feats_undistort_world;
  FastLioPointCloud::Ptr feats_down_body;
  FastLioPointCloud::Ptr feats_down_world;
  FastLioPointCloud::Ptr normvec;
  FastLioPointCloud::Ptr colorvec;
  FastLioPointCloud::Ptr laserCloudOri;
  FastLioPointCloud::Ptr corr_normvect;
  FastLioPointCloud::Ptr corr_colorvect;
  FastLioPointCloud::Ptr _featsArray;

  KD_TREE<FastLioPoint> ikdtree;
  iOctree::Octree ioctree;
  iOctree::Octree ioctree_scan;

  V3F XAxisPoint_body;
  V3F XAxisPoint_world;
  V3D euler_cur;
  V3D position_last;
  V3D Lidar_T_wrt_IMU;
  M3D Lidar_R_wrt_IMU;

  /*** EKF inputs and output ***/
  MeasureGroup Measures;
  esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
  state_ikfom state_point;
  vect3 pos_lid;

  nav_msgs::msg::Path path;
  nav_msgs::msg::Odometry odomAftMapped;
  geometry_msgs::msg::Quaternion geoQuat;
  geometry_msgs::msg::PoseStamped msg_body_pose;

  BoxPointType LocalMap_Points;
  bool Localmap_Initialized = false;

  double timediff_lidar_wrt_imu = 0.0;
  bool timediff_set_flg = false;

  double lidar_mean_scantime = 0.0;
  int scan_num = 0;
  int process_increments = 0;

  FastLioPointCloud::Ptr pcl_wait_pub;
  FastLioPointCloud::Ptr pcl_wait_save;

  shared_ptr<Preprocess> p_pre;
  shared_ptr<ImuProcess> p_imu;
  CamProcessVec p_cams;
};
}  // namespace fastlio