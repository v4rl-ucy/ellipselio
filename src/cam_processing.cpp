#include <cam_processing.h>

CamProcess::CamProcess(int queue_size, std::string cam_topic,
                       rclcpp::Node::SharedPtr node)
    : it_(node), node_(node), img_buffer_(queue_size) {
  cam_sub_ =
      it_.subscribe(cam_topic, queue_size, &CamProcess::CamCallback, this);
}

void CamProcess::SetExtrinsicAndIntrinsic(const V3D &T_cam_lidar,
                                          const M3D &R_cam_lidar,
                                          const V3D &T_imu_lidar,
                                          const M3D &R_imu_lidar,
                                          const M3D &cam_intrinsics) {
  T_cam_lidar_ = T_cam_lidar;
  R_cam_lidar_ = R_cam_lidar;
  T_imu_lidar_ = T_imu_lidar;
  R_imu_lidar_ = R_imu_lidar;
  cam_intrinsics_ = cam_intrinsics;
}

void CamProcess::CamCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
  img_buffer_.push_back(msg);
}

void CamProcess::GetTransform(double time, Pose6D &head, Pose6D &tail, M3D &R,
                              V3D &T) {
  M3D R_imu;
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;

  double dt = time - head.offset_time;

  R_imu << MAT_FROM_ARRAY(head.rot);
  vel_imu << VEC_FROM_ARRAY(head.vel);
  pos_imu << VEC_FROM_ARRAY(head.pos);
  acc_imu << VEC_FROM_ARRAY(tail.acc);
  angvel_avr << VEC_FROM_ARRAY(tail.gyr);

  R = R_imu * Exp(angvel_avr, dt);
  T = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
}

void CamProcess::MatchImageswithIMU(std::vector<Pose6D> &imu_poses,
                                    double pcl_beg_time, double pcl_end_time) {
  double img_time, imu_time;

  matched_imgs_.clear();
  for (auto img_it = img_buffer_.end() - 1; img_it != img_buffer_.begin();
       img_it--) {
    MatchedImg matched_img;

    if (rclcpp::Time((*img_it)->header.stamp).seconds() < pcl_beg_time ||
        rclcpp::Time((*img_it)->header.stamp).seconds() > pcl_end_time) {
      continue;
    }

    img_time = rclcpp::Time((*img_it)->header.stamp).seconds() - pcl_beg_time;
    for (auto imu_it = imu_poses.end() - 1; imu_it != imu_poses.begin();
         imu_it--) {
      auto head = imu_it - 1;
      auto tail = imu_it;
      imu_time = head->offset_time;

      if (imu_time < img_time) {
        matched_img.cv_img =
            cv_bridge::toCvCopy(*img_it, sensor_msgs::image_encodings::BGR8);
        matched_img.head = *head;
        matched_img.tail = *tail;
        matched_imgs_.push_back(matched_img);
        break;
      }
    }
  }
}

void CamProcess::ColorPoint(FastLioPoint &pt, Pose6D &pt_head, Pose6D &pt_tail,
                            double pcl_beg_time) {
  cv::Point2d uv;
  cv::Vec3b color;
  M3D R_pt, R_img;
  V3D T_pt, T_img;
  MatchedImg *matched_img;
  double img_time, pt_time = pt.curvature * 1e-3, diff_time = DBL_MAX;

  for (auto it = matched_imgs_.end() - 1; it != matched_imgs_.begin(); it--) {
    img_time = rclcpp::Time(it->cv_img->header.stamp).seconds() - pcl_beg_time;

    if (fabs(img_time - pt_time) < diff_time) {
      diff_time = fabs(img_time - pt_time);
      matched_img = &(*it);
    } else {
      break;
    }
  }

  if (matched_img == nullptr) {
    return;
  }

  GetTransform(pt_time, pt_head, pt_tail, R_pt, T_pt);
  GetTransform(img_time, matched_img->head, matched_img->tail, R_img, T_img);

  V3D p(pt.x, pt.y, pt.z);
  V3D T_img_pt(T_pt - T_img);
  V3D p_img = R_imu_lidar_.conjugate() *
              (R_img.conjugate() *
                   (R_pt * (R_imu_lidar_ * p + T_imu_lidar_) + T_img_pt) -
               T_imu_lidar_);
  V3D p_cam = R_cam_lidar_ * p_img + T_cam_lidar_;

  uv.x = round((cam_intrinsics_(0) * p_cam(0) / p_cam(2)) + cam_intrinsics_(2));
  uv.y = round((cam_intrinsics_(4) * p_cam(1) / p_cam(2)) + cam_intrinsics_(5));

  if (uv.x >= 0 && uv.x < matched_img->cv_img->image.cols && uv.y >= 0 &&
      uv.y < matched_img->cv_img->image.rows) {
    color = matched_img->cv_img->image.at<cv::Vec3b>(uv.y, uv.x);
    pt.rgb = color[2] << 16 | color[1] << 8 | color[0];
  }
}