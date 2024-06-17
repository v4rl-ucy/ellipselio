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
#include <sensor_msgs/msg/image.hpp>

class CamProcess {
 public:
  CamProcess(int queue_size, std::string cam_topic,
             rclcpp::Node::SharedPtr node);
  void MatchImageswithIMU(std::vector<Pose6D> &imu_poses, double pcl_beg_time,
                          double pcl_end_time);
  void ColorPoint(FastLioPoint &pt, Pose6D &imu_head, Pose6D &imu_tail,
                  double pcl_beg_time);
  void SetExtrinsicAndIntrinsic(const V3D &T_cam_lidar, const M3D &R_cam_lidar,
                                const V3D &T_imu_lidar, const M3D &R_imu_lidar,
                                const M3D &cam_intrinsics);

 private:
  struct MatchedImg {
    Pose6D head;
    Pose6D tail;
    cv_bridge::CvImagePtr cv_img;
  };

  V3D T_cam_lidar_;
  M3D R_cam_lidar_;
  V3D T_imu_lidar_;
  M3D R_imu_lidar_;
  M3D cam_intrinsics_;
  rclcpp::Node::SharedPtr node_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber cam_sub_;
  std::vector<MatchedImg> matched_imgs_;
  boost::circular_buffer<sensor_msgs::msg::Image::ConstSharedPtr> img_buffer_;

  void CamCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
  void GetTransform(double time, Pose6D &head, Pose6D &tail, M3D &R, V3D &T);
};

typedef std::shared_ptr<CamProcess> CamProcessPtr;
typedef std::vector<CamProcessPtr> CamProcessVec;

#endif  // SEE_CAM_PROCESSING_H