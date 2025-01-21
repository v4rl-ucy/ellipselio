#pragma once

#ifndef LIDARPROCESS_H
#define LIDARPROCESS_H

#include <common_lib.h>
#include <common_pcl.h>
#include <ioctree.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

enum LID_TYPE { LIVOX = 1, VELODYNE = 2, OUSTER = 3, HESAI = 4 };

struct LidarParams {
  int type;
  int rate;
  double min_range;
  double max_range;
  double bin_size;
  double map_resolution;
  double map_search_radius;
  double downsample_factor;
  std::string topic;
};

class LidarProcess {
 public:
  ~LidarProcess();
  LidarProcess(LidarParams params, rclcpp::Node::SharedPtr node);
  void ClearPointCloud();
  void GetPointCloud(EllipseLivoPointCloudPtr pc, rclcpp::Time &start_time,
                     rclcpp::Time &end_time, std::vector<int> &bin_pc_sizes,
                     int &start_bin);

  int num_bins_;
  bool lidar_has_data_;
  rclcpp::Time lidar_start_time_, lidar_end_time_;

  std::vector<int> bucket_sizes_;
  std::vector<int> cnt_neighbours_;
  std::vector<int> min_neighbours_;
  std::vector<int> max_neighbours_;
  std::vector<float> search_radii_;
  std::vector<float> octree_resolutions_;

 private:
  void LidarCallback(const sensor_msgs::msg::PointCloud2::UniquePtr msg_in);
  void Process(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void SetMinMaxTime(int bin_idx);
  void ClearBins();

  void SetPoint(LivoxPoint &in_pt, EllipseLivoPoint &out_pt,
                rclcpp::Time &point_time);
  void SetPoint(VelodynePoint &in_pt, EllipseLivoPoint &out_pt,
                rclcpp::Time &point_time);
  void SetPoint(OusterPoint &in_pt, EllipseLivoPoint &out_pt,
                rclcpp::Time &point_time);
  void SetPoint(HesaiPoint &in_pt, EllipseLivoPoint &out_pt,
                rclcpp::Time &point_time);

  template <typename InPtType>
  void ConvertPoint(InPtType &in_pt, EllipseLivoPoint &out_pt,
                    rclcpp::Time &point_time);
  template <typename InPtType>
  void PointCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                         EllipseLivoPointCloudPtr new_pc);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr lidar_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;

  std::vector<int> bin_pcs_sizes_;
  std::vector<std::vector<int>> bin_idxs_;
  std::vector<std::atomic<int>> bin_sizes_;
  std::vector<iOctree::Octree> bin_octrees_;
  std::vector<EllipseLivoPointCloud> bin_pcs_;

  std::vector<rclcpp::Time> bin_min_times_;
  std::vector<rclcpp::Time> bin_max_times_;

  EllipseLivoPointCloudPtr ellipselivo_pc_;

  int start_bin_;
  float mean_range_;
  std::mutex lidar_mutex_;
  LidarParams params_;
};

#endif  // LIDARPROCESS_H
