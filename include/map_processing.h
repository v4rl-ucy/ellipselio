#include <common_lib.h>
#include <common_pcl.h>
#include <imu_processing.h>
#include <ioctree.h>
#include <lidar_processing.h>
#include <math.h>
#include <omp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <project_ellipse.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Core>
#include <chrono>
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

#include "ellipse_lio/msg/ellipse_lio_analytics.hpp"

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

  void timer_callback();
  void init_cam_process();
  void map_incremental(bool init_map);

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

  double map_resolution, map_search_radius;
  int kf_iterations, map_bucket_size, pub_map_n_secs;
  int map_counter = 0, old_map_size = 0, new_map_size = 0, last_map_size = 0;

  int start_bin, num_bins;
  bool initialized = false;

  int ekfom_iter_cnt;
  double max_ekfom_time;
  double ekfom_iter_time;
  Eigen::VectorXd ekfom_data_h;
  Eigen::MatrixXd ekfom_data_h_x;

  std::vector<V3F> mean_sali;
  std::vector<M3F> tensors_p1;
  std::vector<M3F> tensors_p2;
  std::vector<M3F> eigenvectors;
  std::vector<V3F> eigenvalues;
  std::vector<V3F> salivalues;

  std::vector<int> new_neighbours_map_idx;
  std::vector<std::vector<int>> new_neighbours;
  std::vector<std::atomic<int>> new_neighbours_size;

  std::vector<int> mean_cnt;
  std::vector<int> scan_cloud_bins;
  std::vector<int> update_idx;
  std::vector<std::atomic<int>> updated_pt;
  std::vector<int> saliency_idxs;
  std::vector<vector<bool>> filters;
  std::vector<vector<int>> neighbours;

  int num_cams;
  string cam_transport;
  std::vector<string> cam_topics;
  std::vector<double> cam_intrinsics;
  std::vector<double> t_cam_lidars;
  std::vector<double> r_cam_lidars;
  std::vector<long int> cam_frame_rates;

  std::vector<double> t_imu_lidar;
  std::vector<double> r_imu_lidar;

  EllipseLioPointCloudPtr map_cloud;
  EllipseLioPointCloudPtr scan_cloud;
  EllipseLioPointCloudPtr scan_cloud_pub;

  iOctree::Octree ioctree;

  ImuParams imu_params;
  LidarParams lidar_params;

  IkfomSPtr kf_;
  KfState kf_state_, kf_state_pub_;

  ellipse_lio::msg::EllipseLioAnalytics analytics_msg_ =
      ellipse_lio::msg::EllipseLioAnalytics();
  ellipse_lio::msg::EllipseLioAnalytics analytics_msg_pub_ =
      ellipse_lio::msg::EllipseLioAnalytics();

  rclcpp::Time last_pub_time;

  std::shared_ptr<ImuProcess> imu_process;
  std::shared_ptr<LidarProcess> lid_process;
  CamProcessVec cams_process;
};
}  // namespace ellipselio