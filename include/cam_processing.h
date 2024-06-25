#pragma once

#ifndef CAM_PROCESSING_H
#define CAM_PROCESSING_H

#include <common_lib.h>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Eigen>
#include <boost/circular_buffer.hpp>
#include <cfloat>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

class CamProcess {
 public:
  CamProcess(int queue_size, std::string cam_topic,
             rclcpp::Node::SharedPtr node);
  void MatchImageswithIMU(std::vector<Pose6D> &imu_poses, double pcl_beg_time);
  void ColorPoint(FastLioPoint &pt, Pose6D &imu_head, Pose6D &imu_tail,
                  double pcl_beg_time);
  void SetExtrinsicAndIntrinsic(V3D &t_cam_lidar, M3D &R_cam_lidar,
                                V3D &t_imu_lidar, M3D &R_imu_lidar,
                                M3D &cam_intrinsics);
  boost::circular_buffer<sensor_msgs::msg::Image::ConstSharedPtr> img_buffer_;

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
  std::vector<MatchedImg> matched_imgs_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_sub_;

  void CamCallback(const sensor_msgs::msg::Image::UniquePtr msg);
  void GetTransform(double time, Pose6D &head, Pose6D &tail,
                    Eigen::Isometry3d &T_world_imu);
};

typedef std::shared_ptr<CamProcess> CamProcessPtr;
typedef std::vector<CamProcessPtr> CamProcessVec;

#endif  // SEE_CAM_PROCESSING_H