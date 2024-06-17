#pragma once

#ifndef CAM_PROCESSING_H
#define CAM_PROCESSING_H

#include <common_lib.h>
#include <cv_bridge/cv_bridge.h>

#include <boost/circular_buffer.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <cfloat>
#include <cmath>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp">
#include <sensor_msgs/msg/image.hpp>

class CamProcess {
 public:
  CamProcess(int queue_size, std::string cam_topic, std::string cam_info_yaml,
             rclcpp::Node::SharedPtr node);
  void ColorPoint(FastLioPoint &pt, std::vector<Pose6D> &imu_poses,
                  Pose6D &imu_head, Pose6D &imu_tail, double pcl_beg_time,
                  double pcl_end_time);

 private:
  struct CamImg {
    Pose6D head;
    Pose6D tail;
    bool matched = false;
    cv_bridge::CvImagePtr img;
  };

  V3D T_cam_lidar_;
  M3D R_cam_lidar_;
  V3D T_imu_lidar_;
  M3D R_imu_lidar_;
  rclcpp::Node::SharedPtr node_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber cam_sub_;
  boost::circular_buffer<CamImg> img_buffer_;
  camera_info_manager::CameraInfoManager cam_info_;

  void CamCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
  void GetTransform(double time, Pose6D &head, Pose6D &tail, M3D &R, V3D &T);
};

#endif  // SEE_CAM_PROCESSING_H