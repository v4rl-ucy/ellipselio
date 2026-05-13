#ifndef ELLIPSE_LIO_INCLUDE_LIDAR_PROCESSING_H_
#define ELLIPSE_LIO_INCLUDE_LIDAR_PROCESSING_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "common_lib.h"
#include "common_pcl.h"
#include "ioctree.h"

enum class LidType {
  kLivox = 1,
  kVelodyne = 2,
  kOuster = 3,
  kHesai = 4,
  kGazebo = 5,
};

struct LidarParams {
  int type;
  int rate;
  int scan_lines;
  double min_range;
  double max_range;
  double vertical_fov;
  std::string topic;
};

class LidarProcess {
 public:
  ~LidarProcess();
  LidarProcess(LidarParams params, float map_resolution, rclcpp::Node::SharedPtr node);
  void ClearPointCloud();
  bool GetPointCloud(EllipseLioPointCloudPtr pc, rclcpp::Time* start_time, rclcpp::Time* end_time,
                     Eigen::ArrayXi* bin_pcs_sizes, int* start_bin, int* mean_bin);

  int num_bins_;
  bool lidar_has_data_;

  float scan_res_;
  float min_scan_res_;
  float max_search_rad_;
  float max_octree_res_;

  std::atomic<int> lidar_counter_;
  rclcpp::Duration lidar_time_offset_;
  rclcpp::Time lidar_start_time_, lidar_end_time_;

  std::vector<int> bucket_sizes_;
  std::vector<int> cnt_neighbours_;
  std::vector<int> min_neighbours_;
  std::vector<int> max_neighbours_;
  std::vector<float> search_radii_;
  std::vector<float> match_radii_;
  std::vector<float> scan_line_sep_;
  std::vector<float> octree_resolutions_;

 private:
  void LidarCallback(const sensor_msgs::msg::PointCloud2::UniquePtr msg_in);
  void Process(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void SetMinMaxTime(int bin_idx);
  void ClearBins();

  void SetPoint(const LivoxPoint& in_pt0, const LivoxPoint& in_pt, EllipseLioPoint* out_pt,
                rclcpp::Time* point_time);
  void SetPoint(const VelodynePoint& in_pt0, const VelodynePoint& in_pt, EllipseLioPoint* out_pt,
                rclcpp::Time* point_time);
  void SetPoint(const OusterPoint& in_pt0, const OusterPoint& in_pt, EllipseLioPoint* out_pt,
                rclcpp::Time* point_time);
  void SetPoint(const HesaiPoint& in_pt0, const HesaiPoint& in_pt, EllipseLioPoint* out_pt,
                rclcpp::Time* point_time);
  void SetPoint(const GazeboPoint& in_pt0, const GazeboPoint& in_pt, EllipseLioPoint* out_pt,
                rclcpp::Time* point_time);

  template <typename InPtType>
  void ConvertPoint(pcl::PointCloud<InPtType>& in_pc, int pt_idx, rclcpp::Time& point_time);
  template <typename InPtType>
  void PointCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr lidar_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;

  Eigen::ArrayXi bin_pcs_sizes_, bin_pcs_i_;

  std::vector<std::vector<int>> bin_idxs_;
  std::vector<std::atomic<int>> bin_sizes_;
  std::vector<iOctree::Octree> bin_octrees_;
  std::vector<EllipseLioPointCloud> bin_pcs_;

  std::vector<rclcpp::Time> bin_min_times_;
  std::vector<rclcpp::Time> bin_max_times_;

  EllipseLioPointCloudPtr process_pc_;
  EllipseLioPointCloudPtr ellipselio_pc_;

  int start_bin_, mean_bin_;
  double last_lidar_time_;
  std::mutex lidar_mutex_;
  LidarParams params_;
};

#endif  // ELLIPSE_LIO_INCLUDE_LIDAR_PROCESSING_H_
