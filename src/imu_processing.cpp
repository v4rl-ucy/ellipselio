#include <imu_processing.h>

ImuProcess::~ImuProcess() {}

// Setup the imu process
ImuProcess::ImuProcess(IkfomSPtr kf, ImuParams params,
                       rclcpp::Node::SharedPtr node)
    : b_first_frame_(true),
      imu_need_init_(true),
      imu_has_data_(false),
      imu_counter_(0),
      params_(params),
      kf_(kf),
      node_(node),
      imu_states_(params.rate) {
  imu_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions imu_opt;
  imu_opt.callback_group = imu_callback_group_;

  sub_imu_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      params.topic, rclcpp::ServicesQoS(),
      std::bind(&ImuProcess::ImuCallback, this, std::placeholders::_1),
      imu_opt);

  imu_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  imu_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  acc_noise << params.acc_noise, params.acc_noise, params.acc_noise;
  gyr_noise << params.gyr_noise, params.gyr_noise, params.gyr_noise;
  acc_bias << params.acc_bias, params.acc_bias, params.acc_bias;
  gyr_bias << params.gyr_bias, params.gyr_bias, params.gyr_bias;

  Q = process_noise_cov();
  Q.block<3, 3>(0, 0).diagonal() = gyr_noise;
  Q.block<3, 3>(3, 3).diagonal() = acc_noise;
  Q.block<3, 3>(6, 6).diagonal() = gyr_bias;
  Q.block<3, 3>(9, 9).diagonal() = acc_bias;
}

// Callback for imu messages
void ImuProcess::ImuCallback(const sensor_msgs::msg::Imu::UniquePtr msg_in) {
  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
  imu_counter_++;

  if (rclcpp::Time(msg->header.stamp) <= imu_end_time_) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Imu time out of order");
    return;
  }

  imu_mutex_.lock();
  Process(msg);
  imu_mutex_.unlock();
}

// Process the imu message and update the kalman filter state
void ImuProcess::Process(const sensor_msgs::msg::Imu::SharedPtr msg) {
  double dt;
  input_ikfom in;
  ImuState imu_state;
  V3D gyr_avr, acc_avr;
  rclcpp::Time msg_time;

  if (imu_need_init_) {
    InitImu(msg);
    return;
  }

  if (!imu_states_.size()) {
    acc_avr = mean_acc * G_m_s2 / mean_acc.norm();
    gyr_avr = mean_gyr;
  } else {
    gyr_avr << msg->angular_velocity.x, msg->angular_velocity.y,
        msg->angular_velocity.z;
    gyr_avr += imu_states_.back().gyr;
    gyr_avr *= 0.5;
    acc_avr << msg->linear_acceleration.x, msg->linear_acceleration.y,
        msg->linear_acceleration.z;
    acc_avr += imu_states_.back().acc;
    acc_avr *= 0.5 * G_m_s2 / mean_acc.norm();
  }

  msg_time = msg->header.stamp;
  dt = (msg_time - kf_state_.time).seconds();

  in.acc = acc_avr;
  in.gyro = gyr_avr;
  kf_->predict(dt, Q, in);

  kf_state_.state = kf_->get_x();
  kf_state_.cov = kf_->get_P();
  kf_state_.time = msg_time;

  imu_state.state = kf_state_;
  imu_state.acc << msg->linear_acceleration.x, msg->linear_acceleration.y,
      msg->linear_acceleration.z;
  imu_state.gyr << msg->angular_velocity.x, msg->angular_velocity.y,
      msg->angular_velocity.z;
  imu_state.acc_avr = kf_state_.state.rot * (acc_avr - kf_state_.state.ba);
  imu_state.acc_avr += kf_state_.state.grav.get_vect();
  imu_state.gyr_avr = gyr_avr - kf_state_.state.bg;

  imu_states_.push_back(imu_state);
  imu_start_time_ = imu_states_.front().state.time;
  imu_end_time_ = imu_states_.back().state.time;
  imu_has_data_ = true;
}

// Initialize the imu state by averaging the first N imu messages
void ImuProcess::InitImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  if (b_first_frame_) {
    init_iter_num = 1;
    b_first_frame_ = false;
    const auto &imu_acc = msg->linear_acceleration;
    const auto &gyr_acc = msg->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
  } else {
    V3D cur_acc, cur_gyr;
    const auto &imu_acc = msg->linear_acceleration;
    const auto &gyr_acc = msg->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc += cur_acc;
    mean_gyr += cur_gyr;

    init_iter_num++;
  }

  if (init_iter_num > params_.rate) {
    imu_need_init_ = false;
    mean_acc /= init_iter_num;
    mean_gyr /= init_iter_num;

    kf_state_.time = rclcpp::Time(msg->header.stamp);

    kf_state_.state = kf_->get_x();
    kf_state_.state.grav = S2(-mean_acc / mean_acc.norm() * G_m_s2);
    kf_state_.state.bg = mean_gyr;
    kf_state_.state.offset_T_L_I = params_.t_imu_lidar;
    kf_state_.state.offset_R_L_I = params_.r_imu_lidar;
    kf_->change_x(kf_state_.state);

    kf_state_.cov = P_cov();
    kf_->change_P(kf_state_.cov);
  }
}

// Get the closest imu state greater than the input match time
void ImuProcess::GetTimeMatch(int &match_idx, rclcpp::Time &match_time,
                              boost::circular_buffer<ImuState> &imu_states) {
  double time_diff;
  bool match_flag = false;

  time_diff = (match_time - imu_states.front().state.time).seconds();
  match_idx = std::floor(time_diff * params_.rate);
  match_idx = std::max(match_idx, 0);
  match_idx = std::min(match_idx, (int)imu_states.size() - 1);

  while (!match_flag) {
    if (imu_states[match_idx].state.time > match_time) {
      if (match_idx == 0) {
        match_flag = true;
      } else if (imu_states[match_idx - 1].state.time > match_time) {
        match_idx--;
      } else {
        match_flag = true;
      }
    } else if (match_idx == imu_states.size() - 1) {
      match_flag = true;
    } else {
      match_idx++;
    }
  }
}

// Undistort the lidar point cloud using the kalman filter imu states
void ImuProcess::UndistortPointCloud(EllipseLioPointCloudPtr pc,
                                     KfState &kf_state,
                                     rclcpp::Time &lidar_end_time,
                                     CamProcessVec &cams) {
  int match_idx;
  boost::circular_buffer<ImuState> imu_states;
  Eigen::Isometry3d T_imu_lidar, T_world_imu_e;

  imu_mutex_.lock();
  imu_states = imu_states_;
  imu_mutex_.unlock();

  GetMatchingImages(lidar_end_time, cams, imu_states);
  GetTimeMatch(match_idx, lidar_end_time, imu_states);

  kf_state = imu_states[match_idx].state;

  T_imu_lidar.linear() =
      imu_states[match_idx].state.state.offset_R_L_I.toRotationMatrix();
  T_imu_lidar.translation() = imu_states[match_idx].state.state.offset_T_L_I;
  T_world_imu_e.linear() =
      imu_states[match_idx].state.state.rot.toRotationMatrix();
  T_world_imu_e.translation() = imu_states[match_idx].state.state.pos;

#pragma omp parallel for
  for (size_t i = 0; i < pc->points.size(); i++) {
    int head_idx, tail_idx;
    Eigen::Isometry3d T_world_imu_p, T_imu_e_imu_p;

    rclcpp::Time pt_time = rclcpp::Time(pc->points[i].time_secs,
                                        pc->points[i].time_nsecs, RCL_ROS_TIME);

    GetTimeMatch(tail_idx, pt_time, imu_states);
    head_idx = max(tail_idx - 1, 0);

    M3D R_imu = imu_states[head_idx].state.state.rot.toRotationMatrix();
    V3D vel_imu = imu_states[head_idx].state.state.vel;
    V3D pos_imu = imu_states[head_idx].state.state.pos;
    V3D acc_avr = imu_states[tail_idx].acc_avr;
    V3D gyr_avr = imu_states[tail_idx].gyr_avr;

    double dt = (pt_time - imu_states[head_idx].state.time).seconds();

    T_world_imu_p.linear() = R_imu * Exp(gyr_avr, dt);
    T_world_imu_p.translation() =
        pos_imu + vel_imu * dt + 0.5 * acc_avr * dt * dt;
    T_imu_e_imu_p = T_world_imu_e.inverse() * T_world_imu_p;

    ColorisePoint(pc->points[i], cams, T_world_imu_p, T_imu_lidar);

    pc->points[i].getVector3fMap() =
        (T_imu_lidar.inverse() * T_imu_e_imu_p * T_imu_lidar *
         pc->points[i].getVector3fMap().cast<double>())
            .cast<float>();
  }
}

// Compute the camera pose at the time of the closest matching image
void ImuProcess::GetMatchingImages(
    rclcpp::Time &match_time, CamProcessVec &cams,
    boost::circular_buffer<ImuState> &imu_states) {
#pragma omp parallel for
  for (size_t i = 0; i < cams.size(); i++) {
    rclcpp::Time img_time;
    int head_idx, tail_idx;
    Eigen::Isometry3d T_world_img;

    cams[i]->GetMatchingImageTime(match_time, img_time);

    GetTimeMatch(tail_idx, img_time, imu_states);
    head_idx = max(tail_idx - 1, 0);

    M3D R_imu = imu_states[head_idx].state.state.rot.toRotationMatrix();
    V3D vel_imu = imu_states[head_idx].state.state.vel;
    V3D pos_imu = imu_states[head_idx].state.state.pos;
    V3D acc_avr = imu_states[tail_idx].acc_avr;
    V3D gyr_avr = imu_states[tail_idx].gyr_avr;

    double dt = (img_time - imu_states[head_idx].state.time).seconds();

    T_world_img.linear() = R_imu * Exp(gyr_avr, dt);
    T_world_img.translation() =
        pos_imu + vel_imu * dt + 0.5 * acc_avr * dt * dt;

    cams[i]->T_world_img_ = T_world_img;
  }
}

// Colorise a lidar point using the camera images
void ImuProcess::ColorisePoint(EllipseLioPoint &pt, CamProcessVec &cams,
                               Eigen::Isometry3d &T_world_pt,
                               Eigen::Isometry3d &T_imu_lidar) {
  int min_pt_col = 765;

  pt.r = 0;
  pt.g = 0;
  pt.b = 0;
  pt.a = 0;
  pt.has_rgb = false;

  for (size_t i = 0; i < cams.size(); i++) {
    Eigen::Vector3i pt_col;
    Eigen::Vector3d pt_img;

    Eigen::Isometry3d &T_cam_lidar = cams[i]->T_cam_lidar_;
    Eigen::Isometry3d &T_world_img = cams[i]->T_world_img_;

    pt_img = T_cam_lidar * T_imu_lidar.inverse() * T_world_img.inverse() *
             T_world_pt * T_imu_lidar * pt.getVector3fMap().cast<double>();

    if (cams[i]->ColorPoint(pt_img, pt_col)) {
      if (pt_col.sum() < min_pt_col) {
        pt.r = pt_col(0);
        pt.g = pt_col(1);
        pt.b = pt_col(2);
        pt.a = 255;
        pt.has_rgb = true;
        min_pt_col = pt_col.sum();
      }
    }
  }
}

// Update the kalman filter state with the latest lidar point cloud tensor
// registration and recompute newer imu states
void ImuProcess::UpdateStatesWithLidar(KfState &kf_state,
                                       rclcpp::Time &lidar_end_time) {
  int match_idx;
  double solve_time;
  IkfomSPtr kf(new Ikfom());

  imu_mutex_.lock();
  *kf = *kf_;
  imu_mutex_.unlock();

  kf->change_x(kf_state.state);
  kf->change_P(kf_state.cov);
  kf->update_iterated_dyn_share_modified(LIDAR_PT_COV, solve_time);

  imu_mutex_.lock();

  *kf_ = *kf;
  GetTimeMatch(match_idx, lidar_end_time, imu_states_);
  imu_states_[match_idx].state.state = kf_->get_x();
  imu_states_[match_idx].state.cov = kf_->get_P();

  kf_state = imu_states_[match_idx].state;

  for (size_t i = match_idx + 1; i < imu_states_.size(); i++) {
    double dt;
    int head_idx, tail_idx;
    input_ikfom in;
    V3D gyr_avr, acc_avr;

    tail_idx = i;
    head_idx = i - 1;

    gyr_avr = 0.5 * (imu_states_[tail_idx].gyr + imu_states_[head_idx].gyr);
    acc_avr = 0.5 * (imu_states_[tail_idx].acc + imu_states_[head_idx].acc);
    acc_avr *= G_m_s2 / mean_acc.norm();

    dt = (imu_states_[tail_idx].state.time - imu_states_[head_idx].state.time)
             .seconds();

    in.acc = acc_avr;
    in.gyro = gyr_avr;
    kf_->predict(dt, Q, in);

    imu_states_[tail_idx].state.state = kf_->get_x();
    imu_states_[tail_idx].state.cov = kf_->get_P();
    imu_states_[tail_idx].acc_avr =
        imu_states_[tail_idx].state.state.rot *
        (acc_avr - imu_states_[tail_idx].state.state.ba);
    imu_states_[tail_idx].acc_avr +=
        imu_states_[tail_idx].state.state.grav.get_vect();
    imu_states_[tail_idx].gyr_avr =
        gyr_avr - imu_states_[tail_idx].state.state.bg;
  }

  kf_state_.state = imu_states_.back().state.state;
  kf_state_.cov = imu_states_.back().state.cov;
  kf_state_.time = imu_states_.back().state.time;

  imu_mutex_.unlock();
}

// Get the current kalman filter state
void ImuProcess::GetKfState(KfState &kf_state) {
  imu_mutex_.lock();
  kf_state = kf_state_;
  imu_mutex_.unlock();
}
