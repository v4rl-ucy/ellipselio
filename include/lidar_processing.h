#pragma once

#ifndef LIDARPROCESS_H
#define LIDARPROCESS_H

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
  double downsample_factor;
  std::string topic;
};

class LidarProcess {
 public:
  ~LidarProcess();
  LidarProcess(LidarParams params, rclcpp::Node::SharedPtr node);
  void ClearPointCloud();
  void GetPointCloud(EllipseLivoPointCloudPtr pc, rclcpp::Time &end_time,
                     std::vector<int> &num_bin_pts);

  bool lidar_has_data_;
  float scan_min_extent_;
  rclcpp::Time lidar_start_time_, lidar_end_time_;

 private:
  void LidarCallback(const sensor_msgs::msg::PointCloud2::UniquePtr msg_in);
  void Process(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void SetMinMaxTime(EllipseLivoPoint &pt);

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

  std::atomic<float> mean_range_;

  std::vector<int> num_bin_pts_;
  std::vector<std::atomic<int>> bin_size_;
  std::vector<iOctree::Octree> bin_octrees_;
  std::vector<std::vector<int>> bin_idxs_, new_idxs_, added_idxs_;

  EllipseLivoPointCloudPtr ellipselivo_pc_;

  bool upd_lidar_has_data_;
  rclcpp::Time upd_lidar_start_time_, upd_lidar_end_time_;

  std::mutex lidar_mutex_;
  LidarParams params_;
};

#endif  // LIDARPROCESS_H
