#include <cam_processing.h>

CamProcess::CamProcess(int queue_size, std::string cam_topic,
                       rclcpp::Node::SharedPtr node)
    : node_(node), img_buffer_(queue_size) {
  cam_sub_ = image_transport::create_subscription(
      node_.get(), cam_topic,
      std::bind(&CamProcess::CamCallback, this, std::placeholders::_1), "raw",
      rmw_qos_profile_default);
}

void CamProcess::SetExtrinsicAndIntrinsic(V3D &t_cam_lidar, M3D &R_cam_lidar,
                                          V3D &t_imu_lidar, M3D &R_imu_lidar,
                                          M3D &cam_intrinsics) {
  T_cam_lidar_.linear() = R_cam_lidar;
  T_cam_lidar_.translation() = t_cam_lidar;
  T_imu_lidar_.linear() = R_imu_lidar;
  T_imu_lidar_.translation() = t_imu_lidar;
  cam_intrinsics_ = cam_intrinsics;
}

void CamProcess::CamCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  img_buffer_.push_back(msg);
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

void CamProcess::MatchImageswithIMU(std::vector<Pose6D> &imu_poses,
                                    double pcl_beg_time) {
  double img_time, imu_time;

  matched_imgs_.clear();
  if (img_buffer_.empty()) {
    return;
  }
  for (auto img_it = img_buffer_.rbegin(); img_it != img_buffer_.rend();
       img_it++) {
    MatchedImg matched_img;

    img_time = rclcpp::Time((*img_it)->header.stamp).seconds();

    if (img_time < (imu_poses.front().offset_time + pcl_beg_time)) {
      continue;
    }
    if (img_time > (imu_poses.back().offset_time + pcl_beg_time)) {
      continue;
    }

    for (auto imu_it = imu_poses.rbegin(); imu_it != imu_poses.rend() - 1;
         imu_it++) {
      auto head = imu_it + 1;
      auto tail = imu_it;
      imu_time = head->offset_time + pcl_beg_time;

      if (imu_time < img_time) {
        matched_img.cv_img =
            cv_bridge::toCvShare(*img_it, sensor_msgs::image_encodings::BGR8);
        matched_img.head = *head;
        matched_img.tail = *tail;
        matched_imgs_.push_back(matched_img);
        break;
      }
    }
  }

  if (!matched_imgs_.size()) {
    RCLCPP_INFO(node_->get_logger(), "Matched no images with IMU poses");
    RCLCPP_INFO(node_->get_logger(), "Oldest img %f",
                rclcpp::Time(img_buffer_.front()->header.stamp).seconds());
    RCLCPP_INFO(node_->get_logger(), "Newest img %f",
                rclcpp::Time(img_buffer_.back()->header.stamp).seconds());
    RCLCPP_INFO(node_->get_logger(), "Oldest imu %f",
                imu_poses.front().offset_time + pcl_beg_time);
    RCLCPP_INFO(node_->get_logger(), "Newest imu %f",
                imu_poses.back().offset_time + pcl_beg_time);
  }
}

void CamProcess::ColorPoint(FastLioPoint &pt, Pose6D &pt_head, Pose6D &pt_tail,
                            double pcl_beg_time) {
  cv::Point2d uv;
  cv::Vec3b color;
  Eigen::Vector3d pt_cap, pt_img;
  Eigen::Isometry3d T_world_img, T_world_pt, T_img_pt;
  double img_time, matched_img_time, pt_time = pt.curvature * 1e-3,
                                     diff_time = DBL_MAX;

  if (matched_imgs_.empty()) {
    return;
  }
  auto matched_it = matched_imgs_.rbegin();
  for (auto it = matched_imgs_.rbegin(); it != matched_imgs_.rend(); it++) {
    img_time = rclcpp::Time(it->cv_img->header.stamp).seconds() - pcl_beg_time;

    if (fabs(img_time - pt_time) < diff_time) {
      diff_time = fabs(img_time - pt_time);
      matched_it = it;
    } else {
      break;
    }
  }

  matched_img_time =
      rclcpp::Time(matched_it->cv_img->header.stamp).seconds() - pcl_beg_time;
  GetTransform(pt_time, pt_head, pt_tail, T_world_pt);
  GetTransform(matched_img_time, matched_it->head, matched_it->tail,
               T_world_img);

  pt_cap << pt.x, pt.y, pt.z;
  T_img_pt = T_world_img.inverse() * T_world_pt;
  pt_img =
      T_cam_lidar_ * T_imu_lidar_.inverse() * T_img_pt * T_imu_lidar_ * pt_cap;

  uv.x = round((cam_intrinsics_(0, 0) * pt_img(0) / pt_img(2)) +
               cam_intrinsics_(0, 2));
  uv.y = round((cam_intrinsics_(1, 1) * pt_img(1) / pt_img(2)) +
               cam_intrinsics_(1, 2));

  if (uv.x >= 0 && uv.x < matched_it->cv_img->image.cols && uv.y >= 0 &&
      uv.y < matched_it->cv_img->image.rows && pt_img(2) > 0) {
    color = matched_it->cv_img->image.at<cv::Vec3b>(uv.y, uv.x);

    if (pt.r != 0 || pt.g != 0 || pt.b != 0) {
      pt.r = fmin(pt.r, color[2]);
      pt.g = fmin(pt.g, color[1]);
      pt.b = fmin(pt.b, color[0]);
    }
    pt.r = fmax(color[2], 1);
    pt.g = fmax(color[1], 1);
    pt.b = fmax(color[0], 1);
  }
}