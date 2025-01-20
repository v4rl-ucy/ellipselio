#include <cam_processing.h>

CamProcess::CamProcess(CamParams params, rclcpp::Node::SharedPtr node)
    : node_(node), params_(params), img_buffer_(params.rate)) {
  cam_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions cam_opt;
  cam_opt.callback_group = cam_callback_group_;

  cam_sub_ = image_transport::create_subscription(
      node_.get(), params_.topic,
      std::bind(&CamProcess::CamCallback, this, std::placeholders::_1), "raw",
      rmw_qos_profile_sensor_data, cam_opt);

  T_cam_lidar_.linear() = params_.r_cam_lidar;
  T_cam_lidar_.translation() = params_.t_cam_lidar;
  cam_intrinsics_ = params_.cam_intrinsics;

  cam_has_data_ = false;
  img_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  img_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void CamProcess::CamCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  cam_mutex_.lock();
  img_buffer_.push_back(msg);
  img_start_time_ = img_buffer_.front()->header.stamp;
  img_end_time_ = img_buffer_.back()->header.stamp;
  cam_has_data_ = true;
  cam_mutex_.unlock();
}

void CamProcess::GetTransform(double time, Pose6D &head, Pose6D &tail,
                              Eigen::Isometry3d &T_world_imu) {
  M3D R_imu;
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;

  double dt = time - head.offset_time;

  R_imu << MAT_FROM_ARRAY(head.rot);
  vel_imu << VEC_FROM_ARRAY(head.vel);
  pos_imu << VEC_FROM_ARRAY(head.pos);
  acc_imu << VEC_FROM_ARRAY(tail.acc);
  angvel_avr << VEC_FROM_ARRAY(tail.gyr);

  T_world_imu.linear() = R_imu * Exp(angvel_avr, dt);
  T_world_imu.translation() = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
}

void CamProcess::GetMatchingImageTime(rclcpp::Time &match_time,
                                      rclcpp::Time &img_time) {
  int match_idx;
  double time_diff;
  bool match_flag = false;

  cam_mutex_.lock();
  time_diff = (match_time - img_buffer_.front()->header.stamp).seconds();
  match_idx = std::floor(time_diff * params_.rate);
  match_idx = std::min(match_idx, (int)img_buffer_.size() - 1);

  while (!match_flag) {
    if (img_buffer_[match_idx]->header.stamp > match_time) {
      if (match_idx == 0) {
        match_flag = true;
      } else if (img_buffer_[match_idx - 1]->header.stamp > match_time) {
        match_idx--;
      } else {
        match_flag = true;
      }
    } else if (match_idx == img_buffer_.size() - 1) {
      match_flag = true;
    } else {
      match_idx++;
    }
  }

  img_time = img_buffer_[match_idx]->header.stamp;
  matched_img_ = cv_bridge::toCvShare(img_buffer_[match_idx],
                                      sensor_msgs::image_encodings::BGR8);
  cam_mutex_.unlock();
}

bool CamProcess::ColorPoint(V3D &pt_img, V3D &pt_col, float &dist_from_ctr) {
  cv::Point2d uv;
  cv::Vec3b color;

  uv.x = round((cam_intrinsics_(0, 0) * pt_img(0) / pt_img(2)) +
               cam_intrinsics_(0, 2));
  uv.y = round((cam_intrinsics_(1, 1) * pt_img(1) / pt_img(2)) +
               cam_intrinsics_(1, 2));

  if (uv.x >= 0 && uv.x < matched_img_->image.cols && uv.y >= 0 &&
      uv.y < matched_img_->image.rows && pt_img(2) > 0) {
    color = matched_img_->image.at<cv::Vec3b>(uv.y, uv.x);
    dist_from_ctr = sqrt(pow(uv.x - matched_img_->image.cols / 2, 2) +
                         pow(uv.y - matched_img_->image.rows / 2, 2));

    pt_col << color[2], color[1], color[0];
    return true;
  }
  return false;
}