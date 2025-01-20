#pragma once

#ifndef CAM_PROCESSING_H
#define CAM_PROCESSING_H

#include <common_lib.h>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Eigen>
#include <boost/circular_buffer.hpp>
#include <cfloat>
#include <cmath>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

struct CamParams {
  int rate;
  V3D t_cam_lidar;
  M3D r_cam_lidar;
  M3D cam_intrinsics;
  std::string topic;
};

class CamProcess {
 public:
  CamProcess(CamParams params, rclcpp::Node::SharedPtr node);
  void GetMatchingImageTime(rclcpp::Time &match_time, rclcpp::Time &img_time);
  bool ColorPoint(V3D &pt_img, V3D &pt_col, float &dist_from_ctr);

  bool cam_has_data_;
  Eigen::Isometry3d T_cam_lidar_, T_world_img_;
  rclcpp::Time img_start_time_, img_end_time_;
  boost::circular_buffer<sensor_msgs::msg::Image::ConstSharedPtr> img_buffer_;

 private:
  CamParams params_;
  std::mutex cam_mutex_;

  Eigen::Matrix3d cam_intrinsics_;
  cv_bridge::CvImageConstPtr matched_img_;

  rclcpp::Node::SharedPtr node_;
  image_transport::Subscriber cam_sub_;
  rclcpp::CallbackGroup::SharedPtr cam_callback_group_;

  void CamCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
};

typedef std::shared_ptr<CamProcess> CamProcessPtr;
typedef std::vector<CamProcessPtr> CamProcessVec;

#endif  // SEE_CAM_PROCESSING_H