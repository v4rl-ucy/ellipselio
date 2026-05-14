/**
 * @file cam_processing.h
 * @brief Camera image processing and color projection to LiDAR points.
 * @details Handles synchronized image buffer management, time matching with
 * LiDAR, and bilinear color interpolation for point cloud colorization.
 */

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

/**
 * @struct CamParams
 * @brief Camera sensor parameters and extrinsic calibration.
 * @details Stores camera frame rate, calibration data (intrinsics), and the
 * rigid transformation between camera and LiDAR coordinate frames.
 */
struct CamParams {
  /// @brief Image frame rate in Hz
  int rate;
  /// @brief Translation from LiDAR to camera frame [m]
  V3D t_cam_lidar;
  /// @brief Rotation matrix from LiDAR to camera frame
  M3D r_cam_lidar;
  /// @brief Camera intrinsic matrix K [fx, 0, cx; 0, fy, cy; 0, 0, 1]
  M3D cam_intrinsics;
  /// @brief ROS 2 topic name for image subscription
  std::string topic;
  /// @brief Image transport type (e.g., "raw", "compressed")
  std::string transport;
};

/**
 * @struct Img
 * @brief Timestamped image with ROS header information.
 */
struct Img {
  /// @brief Image acquisition timestamp
  rclcpp::Time time;
  /// @brief OpenCV image data (BGR color space)
  cv_bridge::CvImageConstPtr img;
};

/**
 * @class CamProcess
 * @brief Processes camera images and projects color onto LiDAR points.
 * @details Maintains a circular buffer of synchronized images, provides
 * time-matching queries to find images corresponding to LiDAR timestamps, and
 * performs bilinear color interpolation to colorize 3D points.
 */
class CamProcess {
 public:
  /**
   * @brief Construct a camera processor with calibration parameters.
   * @param params Camera parameters (intrinsics, extrinsics, frame rate)
   * @param node Shared pointer to ROS 2 node for subscription and logging
   */
  CamProcess(CamParams params, rclcpp::Node::SharedPtr node);

  /**
   * @brief Find image closest in time to a LiDAR frame.
   * @param match_time Target timestamp (typically LiDAR scan time)
   * @param img_time Output: timestamp of matched image
   * @details Performs binary search in circular buffer to find the image with
   *          timestamp closest to @p match_time. Sets @c has_img_match_ flag.
   */
  void GetMatchingImageTime(const rclcpp::Time& match_time,
                            rclcpp::Time* img_time);

  /**
   * @brief Extract and interpolate color for a 3D point projected to image.
   * @param pt_img Pointer to 3D point in camera coordinates [x, y, z]
   * @param pt_col Output: BGR color vector [B, G, R] (OpenCV convention)
   * @return True if point is within image bounds and color retrieved; false
   * otherwise
   * @details Performs bilinear interpolation for subpixel accuracy. Returns
   * false if point is behind camera (z <= 0) or outside image boundaries.
   */
  bool ColorPoint(V3D* pt_img, Eigen::Vector3i* pt_col);

  /// @brief Whether any camera images have been received
  bool cam_has_data_;
  /// @brief Whether a matching image was found for current LiDAR frame
  bool has_img_match_;

  /// @brief Frame counter (incremented per received image)
  std::atomic<int> cam_counter_;
  /// @brief Transformation from camera to LiDAR frame
  Eigen::Isometry3d T_cam_lidar_;
  /// @brief Transformation from world to camera frame (updated per frame)
  Eigen::Isometry3d T_world_img_;
  /// @brief Earliest timestamp in current image buffer
  rclcpp::Time img_start_time_;
  /// @brief Latest timestamp in current image buffer
  rclcpp::Time img_end_time_;

 private:
  /// @brief Camera configuration parameters
  CamParams params_;
  /// @brief Mutex for thread-safe image buffer access
  std::mutex cam_mutex_;

  /// @brief Most recently matched image with timestamp
  Img matched_img_;
  /// @brief Precomputed camera intrinsic matrix for faster projection
  Eigen::Matrix3d cam_intrinsics_;

  /// @brief Shared pointer to ROS 2 node
  rclcpp::Node::SharedPtr node_;
  /// @brief Subscriber for image_transport camera topic
  image_transport::Subscriber cam_sub_;
  /// @brief Callback group for image processing
  rclcpp::CallbackGroup::SharedPtr cam_callback_group_;
  /// @brief Circular buffer of received images (FIFO ordered by timestamp)
  boost::circular_buffer<Img> img_buffer_;

  /// @brief Timestamp of most recently processed image
  double last_cam_time_;

  /**
   * @brief ROS 2 callback for image reception.
   * @param msg Incoming image message (converted to OpenCV Mat)
   * @details Adds image to circular buffer and updates @c last_cam_time_.
   */
  void CamCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
};

/// @brief Shared pointer to CamProcess
using CamProcessPtr = std::shared_ptr<CamProcess>;
/// @brief Vector of shared pointers to multiple camera processors
using CamProcessVec = std::vector<CamProcessPtr>;

#endif  // CAM_PROCESSING_H_