#pragma once

#ifndef CAM_PROCESSING_H
#define CAM_PROCESSING_H

#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Eigen>

class CamProcess {
 public:
  CamProcess();
  ~CamProcess();

 private:
  ros::Subscriber sub_cam;
  Eigen::Isometry3d T_cam_lidar;
  Eigen::Vector
};

#endif  // SEE_CAM_PROCESSING_H