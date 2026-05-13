#ifndef CAM_PROCESSING_H_
#define CAM_PROCESSING_H_

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

#include "common_lib.h"

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
  void GetMatchingImageTime(const rclcpp::Time& match_time,
                            rclcpp::Time* img_time);
  bool ColorPoint(V3D* pt_img, Eigen::Vector3i* pt_col);

  bool cam_has_data_;
  bool has_img_match_;

  std::atomic<int> cam_counter_;
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

  double last_cam_time_;

  void CamCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
};

using CamProcessPtr = std::shared_ptr<CamProcess>;
using CamProcessVec = std::vector<CamProcessPtr>;

#endif  // CAM_PROCESSING_H_