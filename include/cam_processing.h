#pragma once

#ifndef CAM_PROCESSING_H
#define CAM_PROCESSING_H

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

class CamProcess {
 public:
  CamProcess();

 private:
  image_transport::ImageTransport it_;
  image_transport::Subscriber cam_sub_;
  Eigen::Isometry3d T_cam_lidar_;

  void CamCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
};

#endif  // SEE_CAM_PROCESSING_H