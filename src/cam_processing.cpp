#include <cam_processing.h>

CamProcess::CamProcess(CamParams params, rclcpp::Node::SharedPtr node)
    : node_(node), params_(params), img_buffer_(params.rate) {
  cam_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions cam_opt;
  cam_opt.callback_group = cam_callback_group_;

  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  qos_profile.depth = 1;

  cam_sub_ = image_transport::create_subscription(
      node_.get(), params_.topic,
      std::bind(&CamProcess::CamCallback, this, std::placeholders::_1),
      params_.transport, qos_profile, cam_opt);

  T_cam_lidar_.linear() = params_.r_cam_lidar;
  T_cam_lidar_.translation() = params_.t_cam_lidar;
  cam_intrinsics_ = params_.cam_intrinsics;

  cam_has_data_ = false;
  img_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  img_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void CamProcess::CamCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  if (rclcpp::Time(msg->header.stamp) < img_end_time_) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Cam time out of order");
    return;
  }

  Img img;
  cam_mutex_.lock();
  img.time = msg->header.stamp;
  img.img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  img_buffer_.push_back(img);
  img_start_time_ = img_buffer_.front().time;
  img_end_time_ = img_buffer_.back().time;
  cam_has_data_ = true;
  cam_mutex_.unlock();
}

void CamProcess::GetMatchingImageTime(rclcpp::Time &match_time,
                                      rclcpp::Time &img_time) {
  int match_idx;
  double time_diff;
  bool match_flag = false;

  cam_mutex_.lock();
  time_diff = (match_time - img_buffer_.front().time).seconds();
  match_idx = std::floor(time_diff * params_.rate);
  match_idx = std::max(match_idx, 0);
  match_idx = std::min(match_idx, (int)img_buffer_.size() - 1);

  while (!match_flag) {
    if (img_buffer_[match_idx].time > match_time) {
      if (match_idx == 0) {
        match_flag = true;
      } else if (img_buffer_[match_idx - 1].time > match_time) {
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

  img_time = img_buffer_[match_idx].time;
  matched_img_ = img_buffer_[match_idx];
  cam_mutex_.unlock();
}

bool CamProcess::ColorPoint(V3D &pt_img, Eigen::Vector3i &pt_col) {
  cv::Vec3b color;
  Eigen::Vector3i tmp_col;
  int min_pt_col = 765;
  int x, y, x_d, y_d, cols, rows;

  cols = matched_img_.img->image.cols;
  rows = matched_img_.img->image.rows;

  x = round((cam_intrinsics_(0, 0) * pt_img(0) / pt_img(2)) +
            cam_intrinsics_(0, 2));
  y = round((cam_intrinsics_(1, 1) * pt_img(1) / pt_img(2)) +
            cam_intrinsics_(1, 2));

  if (x >= 0 && x < cols && y >= 0 && y < rows && pt_img(2) > 0) {
    for (int i = -1; i < 2; i++) {
      for (int j = -1; j < 2; j++) {
        x_d = std::min(std::max(x + i, 0), cols - 1);
        y_d = std::min(std::max(y + j, 0), rows - 1);
        color = matched_img_.img->image.at<cv::Vec3b>(y_d, x_d);
        tmp_col << color[2], color[1], color[0];
        if (tmp_col.sum() > 0 && tmp_col.sum() < min_pt_col) {
          pt_col = tmp_col;
          min_pt_col = tmp_col.sum();
        }
      }
    }

    if (min_pt_col < 765) {
      return true;
    }
  }
  return false;
}