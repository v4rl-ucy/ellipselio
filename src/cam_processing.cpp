#include "cam_processing.h"

CamProcess::CamProcess(CamParams params, rclcpp::Node::SharedPtr node)
    : node_(node), params_(params), img_buffer_(params.rate), cam_counter_(0) {
  cam_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions cam_opt;
  cam_opt.callback_group = cam_callback_group_;

  T_cam_lidar_.linear() = params_.r_cam_lidar;
  T_cam_lidar_.translation() = params_.t_cam_lidar;
  cam_intrinsics_ = params_.cam_intrinsics;

  cam_has_data_ = false;
  img_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  img_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  cam_sub_ = image_transport::create_subscription(
      node_.get(), params_.topic,
      std::bind(&CamProcess::CamCallback, this, std::placeholders::_1),
      params_.transport, rmw_qos_profile_sensor_data, cam_opt);
}

void CamProcess::CamCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  cam_counter_++;

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

void CamProcess::GetMatchingImageTime(const rclcpp::Time& match_time,
                                      rclcpp::Time* img_time) {
  int match_idx;
  double time_diff;
  bool match_flag = false;

  has_img_match_ = false;
  if (!img_buffer_.size()) return;

  cam_mutex_.lock();

  time_diff = (match_time - img_buffer_.front().time).seconds();
  match_idx = std::floor(time_diff * params_.rate);
  match_idx = std::max(match_idx, 0);
  match_idx = std::min(match_idx, static_cast<int>(img_buffer_.size()) - 1);

  while (!match_flag && match_idx >= 0) {
    if (img_buffer_[match_idx].time < match_time) {
      if (match_idx == img_buffer_.size() - 1) {
        match_flag = true;
      } else if (img_buffer_[match_idx + 1].time < match_time) {
        match_idx++;
      } else {
        match_flag = true;
      }
    } else if (match_idx == 0) {
      match_flag = true;
    } else {
      match_idx--;
    }
  }

  if (match_idx >= 0) {
    has_img_match_ = true;
    *img_time = img_buffer_[match_idx].time;
    matched_img_ = img_buffer_[match_idx];
  }

  cam_mutex_.unlock();
}

bool CamProcess::ColorPoint(V3D* pt_img, Eigen::Vector3i* pt_col) {
  float x, y;
  int cols, rows, valid_num;
  Eigen::MatrixXi pt_cols;
  Eigen::VectorXi pt_sum;
  Eigen::Vector2i x_vals, y_vals;

  if ((*pt_img)(2) <= 0) return false;

  pt_sum = Eigen::VectorXi::Zero(4);
  pt_cols = Eigen::MatrixXi::Zero(4, 3);

  cols = matched_img_.img->image.cols;
  rows = matched_img_.img->image.rows;

  x = (cam_intrinsics_(0, 0) * (*pt_img)(0) / (*pt_img)(2)) +
      cam_intrinsics_(0, 2);
  y = (cam_intrinsics_(1, 1) * (*pt_img)(1) / (*pt_img)(2)) +
      cam_intrinsics_(1, 2);

  x_vals << std::floor(x), std::ceil(x);
  y_vals << std::floor(y), std::ceil(y);

#pragma omp parallel for
  for (int i = 0; i < 2; i++) {
#pragma omp parallel for
    for (int j = 0; j < 2; j++) {
      int x_i = x_vals(i);
      int y_j = y_vals(j);

      if (x_i >= 0 && x_i < cols && y_j >= 0 && y_j < rows) {
        cv::Vec3b color = matched_img_.img->image.at<cv::Vec3b>(y_j, x_i);
        pt_cols(i * 2 + j, 0) = color[2];
        pt_cols(i * 2 + j, 1) = color[1];
        pt_cols(i * 2 + j, 2) = color[0];
        if (pt_cols.row(i * 2 + j).sum()) {
          pt_sum(i * 2 + j) = 1;
        }
      }
    }
  }

  *pt_col = pt_cols.colwise().sum();
  valid_num = pt_sum.sum();

  if (!valid_num) return false;

  *pt_col /= valid_num;
  return true;
}