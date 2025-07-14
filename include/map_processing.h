#include <common_lib.h>
#include <common_pcl.h>
#include <ellipsoid_harmonics.h>
#include <imu_processing.h>
#include <ioctree.h>
#include <lidar_processing.h>
#include <math.h>
#include <omp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <project_ellipse.h>
#include <sys/times.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Core>
#include <chrono>
#include <ellipse_lio/msg/ellipse_lio_analytics.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace ellipselio {

class MappingNode : public rclcpp::Node {
 public:
  MappingNode(const rclcpp::NodeOptions &options);
  ~MappingNode();

 private:
  bool sync_packages();

  void compute_tensor_vote(int i, int j, M3F &A_j, bool first_pass);
  void compute_tensor_eigen(int i, M3F &tensor, bool first_pass);

  void tensor_vote_pass_1(int old_map_size, std::vector<int> &added_idxs,
                          std::vector<int> &updated_idxs);
  void tensor_vote_pass_2(std::vector<int> &added_idxs,
                          std::vector<int> &updated_idxs);

  void compute_harmonics(int map_i, int map_j, int loop_idx, SHCoeffs &SH);
  void compute_geometric_primitive(int map_i, int sali_idx, V3F &p_world,
                                   V3F &norm_vec);

  void split_map(const sensor_msgs::msg::PointCloud2 &input,
                 std::vector<sensor_msgs::msg::PointCloud2> &clouds, size_t n);
  void publish_map();
  void publish_scan();
  void publish_markers();
  void publish_odometry();

  void tensor_registration(state_ikfom &s,
                           esekfom::dyn_share_datastruct<double> &ekfom_data);
  void zero_registration_values();

  void timer_callback();
  void init_cam_process();
  void map_incremental();

  void compute_ram_usage();
  void compute_cpu_usage();

  void sync_raw_cloud_with_imu();

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_mark_;
  rclcpp::Publisher<ellipse_lio::msg::EllipseLioAnalytics>::SharedPtr
      pub_analytics_;

  rclcpp::TimerBase::SharedPtr loop_timer_;
  rclcpp::TimerBase::SharedPtr pub_odo_timer_;
  rclcpp::TimerBase::SharedPtr pub_map_timer_;
  rclcpp::CallbackGroup::SharedPtr pub_map_callback_group_;
  rclcpp::CallbackGroup::SharedPtr pub_odo_callback_group_;
  rclcpp::CallbackGroup::SharedPtr loop_callback_group_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;

  double last_sync_time = 0, max_imu_time = 0, max_state_time = 0,
         max_map_time = 0, max_total_time = 0, mean_imu_time = 0,
         mean_state_time = 0, mean_map_time = 0, mean_total_time = 0;

  int pub_map_n_secs;
  int map_counter = 0, old_map_size = 0, new_map_size = 0, last_map_size = 0;

  double start_time;
  bool initialized = false;
  bool use_map_res = false;
  bool line_sep_res = true;

  int scan_num_cnt = 0;
  double map_resolution, map_search_rad;
  int start_bin, mean_bin, max_mean_bin = 0;

  int numProcessors;
  clock_t lastCPU, lastSysCPU, lastUserCPU;

  int ekfom_iter_cnt;
  int ekfom_upd_cnt = 0;
  int feats_per_bin = 0;

  Eigen::ArrayXf n_res;
  Eigen::ArrayXi n_means;
  Eigen::ArrayXXi n_cnts;
  Eigen::ArrayXXi n_bins;
  Eigen::Array3i max_prim_cnts;

  Eigen::ArrayXXi ekfom_data_i;
  Eigen::ArrayXXd ekfom_data_v;
  Eigen::ArrayXXd ekfom_data_w;
  Eigen::VectorXd ekfom_data_h;
  Eigen::ArrayXd ekfom_data_w_x;
  Eigen::MatrixXd ekfom_data_h_x;
  Eigen::MatrixXd ekfom_data_h_x_R;
  std::vector<Eigen::ArrayXd> ekfom_data_h_v;
  std::vector<Eigen::MatrixXd> ekfom_data_h_x_v;

  std::vector<Eigen::Vector3f> colors;
  std::vector<Eigen::MatrixXf> sh_mats;

  std::vector<Eigen::Vector3f> poses;
  std::vector<Eigen::Quaternionf> rotes;

  std::vector<bool> init_poses;
  std::vector<Eigen::Vector3f> last_updated_poses;
  std::vector<Eigen::Quaternionf> last_updated_rotes;

  std::vector<M3F> tensors_p1;
  std::vector<M3F> tensors_p2;
  std::vector<M3F> eigenvectors;
  std::vector<V3F> eigenvalues;
  std::vector<V3F> salivalues;

  Eigen::ArrayXi raw_cloud_bins;
  Eigen::ArrayXi scan_cloud_bins;
  Eigen::ArrayXi buffer_cloud_bins;
  std::vector<std::atomic<int>> scan_bin_sizes;
  std::vector<std::atomic<int>> filter_bin_sizes;

  std::vector<int> new_neighbours_map_idx;
  std::vector<std::atomic<int>> updated_pt;
  std::vector<std::vector<int>> new_neighbours;
  std::vector<std::atomic<int>> new_neighbours_size;

  std::vector<int> valid_reg;
  std::vector<int> update_idx;
  std::vector<int> saliency_idxs;
  std::vector<vector<int>> neighbours;
  std::vector<Eigen::Vector2i> filters;

  int num_cams;
  string cam_transport;
  std::vector<string> cam_topics;
  std::vector<double> t_cam_lidars;
  std::vector<double> r_cam_lidars;
  std::vector<double> cam_intrinsics;
  std::vector<long int> cam_frame_rates;

  std::vector<double> t_imu_lidar;
  std::vector<double> r_imu_lidar;

  EllipseLioPointCloudPtr map_cloud;
  EllipseLioPointCloudPtr raw_cloud;
  EllipseLioPointCloudPtr scan_cloud;
  EllipseLioPointCloudPtr filter_cloud;
  EllipseLioPointCloudPtr buffer_cloud;
  EllipseLioPointCloudPtr scan_cloud_pub;

  iOctree::Octree ioctree;

  ImuParams imu_params;
  LidarParams lidar_params;

  IkfomSPtr kf_;
  KfState kf_state_, kf_state_pub_;

  std::mutex map_mutex_;
  std::mutex odom_mutex_;

  ellipse_lio::msg::EllipseLioAnalytics analytics_msg_;
  ellipse_lio::msg::EllipseLioAnalytics analytics_msg_pub_;

  rclcpp::Time last_pub_time, last_imu_time_;
  rclcpp::Time imu_start_time_, imu_end_time_;
  rclcpp::Time raw_start_time_, raw_end_time_;
  rclcpp::Time scan_start_time_, scan_end_time_;
  rclcpp::Time buffer_start_time_, buffer_end_time_;

  double imu_time_offset_ = 0, lid_time_offset_ = 0;

  CamProcessVec cams_process;
  std::shared_ptr<ImuProcess> imu_process;
  std::shared_ptr<LidarProcess> lid_process;
  std::shared_ptr<EllipsoidHarmonics> harmonics;
};
}  // namespace ellipselio