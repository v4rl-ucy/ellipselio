#include <cam_processing.h>

CamProcess::CamProcess(int queue_size, std::string cam_topic,
                       std::string cam_info_yaml, rclcpp::Node::SharedPtr node)
    : it_(node), node_(node), img_buffer_(queue_size) {
  cam_sub_ =
      it_.subscribe(cam_topic, queue_size, &CamProcess::CamCallback, this);
  cam_info_ = camera_info_manager::CameraInfoManager(node_);

  if (cam_info_.validateURL(cam_info_yaml)) {
    cam_info_.loadCameraInfo(cam_info_yaml);
  } else {
    RCLCPP_WARN(node_->get_logger(), "CameraInfo URL not valid.");
    RCLCPP_WARN(node_->get_logger(), "URL IS %s", cam_info_yaml.c_str());
  }
}

void CamProcess::CamCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
  CamImg cam_img;
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cam_img.img = cv_ptr;
    img_buffer_.push_back(cam_img);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
}

void CamProcess::GetTransform(double time, Pose6D &head, Pose6D &tail, M3D &R,
                              V3D &T) {
  M3D R_imu;
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;

  dt = time - head->offset_time;

  R_imu << MAT_FROM_ARRAY(head->rot);
  vel_imu << VEC_FROM_ARRAY(head->vel);
  pos_imu << VEC_FROM_ARRAY(head->pos);
  acc_imu << VEC_FROM_ARRAY(tail->acc);
  angvel_avr << VEC_FROM_ARRAY(tail->gyr);

  R = R_imu * Exp(angvel_avr, dt);
  T = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
}

void CamProcess::ColorPoint(FastLioPoint &pt, std::vector<Pose6D> &imu_poses,
                            Pose6D &pt_head, Pose6D &pt_tail,
                            double pcl_beg_time, double pcl_end_time) {
  cv::Point2d uv;
  cv::Vec3b color;
  CamImg cam_img;
  M3D R_pt, R_img;
  V3D T_pt, T_img;
  double pt_time = pt->curvature * 1e-3;
  double imu_time, img_time, diff_time = DBL_MAX;

  for (auto it = img_buffer_.end() - 1; it != img_buffer_.begin(); it--) {
    img_time = it->img->header.stamp.toSec() - pcl_beg_time;
    if (img_time > pcl_end_time) {
      continue;
    }
    if (img_time < pcl_beg_time) {
      break;
    }
    if (fabs(img_time - pt_time) < diff_time) {
      diff_time = fabs(img_time - pt_time);
      cam_img = *it;
    } else {
      break;
    }
  }

  img_time = cam_img.img->header.stamp.toSec() - pcl_beg_time;
  for (auto it = imu_poses.end() - 1; it != imu_poses.begin(); it--) {
    auto head = it - 1;
    auto tail = it;
    imu_time = head->offset_time;

    if (cam_img.matched) {
      break;
    }
    if (imu_time < img_time) {
      cam_img.head = *head;
      cam_img.tail = *tail;
      break;
    }
  }

  GetTransform(pt_time, pt_head, pt_tail, R_pt, T_pt);
  GetTransform(img_time, cam_img.head, cam_img.tail, R_img, T_img);

  V3D p(pt->x, pt->y, pt->z);
  V3D T_img_pt(T_pt - T_cam);
  V3D p_img = R_imu_lidar_.conjugate() *
              (R_img.conjugate() *
                   (R_pt * (R_imu_lidar_ * p + T_imu_lidar_) + T_img_pt) -
               T_imu_lidar_);
  V3D p_cam = R_cam_lidar_ * p_img + T_cam_lidar_;

  uv.x = round((cam_info_.K[0] * p_cam(0) / p_cam(2)) + cam_info_.K[2]);
  uv.y = round((cam_info_.K[4] * p_cam(1) / p_cam(2)) + cam_info_.K[5]);

  if (uv.x >= 0 || uv.x < cam_img.img->cols || uv.y >= 0 ||
      uv.y < cam_img.img->rows) {
    color = cam_img.img->at<cv::Vec3b>(uv.y, uv.x);
    pt->rgb = color[2] << 16 | color[1] << 8 | color[0];
  }
}