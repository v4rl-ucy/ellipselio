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
  V3D t_imu_lidar;
  M3D r_imu_lidar;
  V3D t_cam_lidar;
  M3D r_cam_lidar;
  M3D cam_intrinsics;
  std::string topic;
};

class CamProcess {
 public:
  CamProcess(CamParams params, rclcpp::Node::SharedPtr node);
  void MatchImageswithIMU(std::vector<Pose6D> &imu_poses, double pcl_beg_time);
  void ColorPoint(EllipseLivoPoint &pt, Pose6D &imu_head, Pose6D &imu_tail,
                  double pcl_beg_time);

  boost::circular_buffer<sensor_msgs::msg::Image::ConstSharedPtr> img_buffer_;
  rclcpp::Time img_start_time_, img_end_time_;

 private:
  struct MatchedImg {
    Pose6D head;
    Pose6D tail;
    cv_bridge::CvImageConstPtr cv_img;
  };

  Eigen::Isometry3d T_cam_lidar_;
  Eigen::Isometry3d T_imu_lidar_;
  Eigen::Matrix3d cam_intrinsics_;

  rclcpp::Node::SharedPtr node_;
  image_transport::Subscriber cam_sub_;
  std::vector<MatchedImg> matched_imgs_;

  void CamCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void GetTransform(double time, Pose6D &head, Pose6D &tail,
                    Eigen::Isometry3d &T_world_imu);
};

typedef std::shared_ptr<CamProcess> CamProcessPtr;
typedef std::vector<CamProcessPtr> CamProcessVec;

#endif  // SEE_CAM_PROCESSING_H