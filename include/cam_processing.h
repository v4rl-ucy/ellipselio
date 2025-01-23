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
  std::string transport;
};

struct Img {
  rclcpp::Time time;
  cv_bridge::CvImageConstPtr img;
};

class CamProcess {
 public:
  CamProcess(CamParams params, rclcpp::Node::SharedPtr node);
  void GetMatchingImageTime(rclcpp::Time &match_time, rclcpp::Time &img_time);
  bool ColorPoint(V3D &pt_img, Eigen::Vector3i &pt_col);

  bool cam_has_data_;
  Eigen::Isometry3d T_cam_lidar_, T_world_img_;
  rclcpp::Time img_start_time_, img_end_time_;

 private:
  CamParams params_;
  std::mutex cam_mutex_;

  Img matched_img_;
  Eigen::Matrix3d cam_intrinsics_;

  rclcpp::Node::SharedPtr node_;
  image_transport::Subscriber cam_sub_;
  rclcpp::CallbackGroup::SharedPtr cam_callback_group_;
  boost::circular_buffer<Img> img_buffer_;

  void CamCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
};

typedef std::shared_ptr<CamProcess> CamProcessPtr;
typedef std::vector<CamProcessPtr> CamProcessVec;

#endif  // SEE_CAM_PROCESSING_H