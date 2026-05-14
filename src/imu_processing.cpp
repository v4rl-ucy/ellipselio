#include "imu_processing.h"

ImuProcess::~ImuProcess() {}

ImuProcess::ImuProcess(IkfomSPtr kf, ImuParams params,
                       rclcpp::Node::SharedPtr node)
    : b_first_frame_(true),
      imu_need_init_(true),
      imu_has_data_(false),
      lidar_ready_(false),
      imu_counter_(0),
      params_(params),
      kf_(kf),
      node_(node),
      imu_states_(params.rate) {
  imu_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions imu_opt;
  imu_opt.callback_group = imu_callback_group_;

  imu_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  imu_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  acc_noise_ << params.acc_noise, params.acc_noise, params.acc_noise;
  gyr_noise_ << params.gyr_noise, params.gyr_noise, params.gyr_noise;
  acc_bias_ << params.acc_bias, params.acc_bias, params.acc_bias;
  gyr_bias_ << params.gyr_bias, params.gyr_bias, params.gyr_bias;

  q_ = ProcessNoiseCov();
  q_.block<3, 3>(0, 0).diagonal() = gyr_noise_;
  q_.block<3, 3>(3, 3).diagonal() = acc_noise_;
  q_.block<3, 3>(6, 6).diagonal() = gyr_bias_;
  q_.block<3, 3>(9, 9).diagonal() = acc_bias_;

  sub_imu_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      params.topic, rclcpp::SensorDataQoS(),
      std::bind(&ImuProcess::ImuCallback, this, std::placeholders::_1),
      imu_opt);
}

void ImuProcess::ImuCallback(const sensor_msgs::msg::Imu::UniquePtr msg_in) {
  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
  imu_counter_++;

  if (rclcpp::Time(msg->header.stamp) < imu_end_time_) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Imu time out of order");
    return;
  }

  imu_mutex_.lock();
  Process(msg);
  imu_mutex_.unlock();
}

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
    acc_avr = mean_acc_ * kGravityMetersPerSecondSquared / mean_acc_.norm();
    gyr_avr = mean_gyr_;
  } else {
    gyr_avr << msg->angular_velocity.x, msg->angular_velocity.y,
        msg->angular_velocity.z;
    gyr_avr += imu_states_.back().gyr;
    gyr_avr *= 0.5;
    acc_avr << msg->linear_acceleration.x, msg->linear_acceleration.y,
        msg->linear_acceleration.z;
    acc_avr += imu_states_.back().acc;
    acc_avr *= 0.5 * kGravityMetersPerSecondSquared / mean_acc_.norm();
  }

  msg_time = msg->header.stamp;
  dt = (msg_time - kf_state_.time).seconds();

  in.acc = acc_avr;
  in.gyro = gyr_avr;
  kf_->predict(dt, q_, in);

  kf_state_.state = kf_->get_x();
  kf_state_.cov = kf_->get_P();
  kf_state_.time = msg_time;
  kf_state_.gyr << msg->angular_velocity.x, msg->angular_velocity.y,
      msg->angular_velocity.z;

  SetKfState();

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

void ImuProcess::InitImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  if (b_first_frame_) {
    init_iter_num_ = 1;
    b_first_frame_ = false;
    const auto& imu_acc = msg->linear_acceleration;
    const auto& gyr_vel = msg->angular_velocity;
    mean_acc_ << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr_ << gyr_vel.x, gyr_vel.y, gyr_vel.z;
  } else {
    V3D cur_acc, cur_gyr;
    const auto& imu_acc = msg->linear_acceleration;
    const auto& gyr_vel = msg->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_vel.x, gyr_vel.y, gyr_vel.z;

    mean_acc_ += cur_acc;
    mean_gyr_ += cur_gyr;

    init_iter_num_++;
  }

  if (init_iter_num_ > params_.rate && lidar_ready_) {
    imu_need_init_ = false;
    mean_acc_ /= init_iter_num_;
    mean_gyr_ /= init_iter_num_;

    kf_state_.time = rclcpp::Time(msg->header.stamp);

    kf_state_.state = kf_->get_x();
    kf_state_.state.grav =
        S2(-mean_acc_ / mean_acc_.norm() * kGravityMetersPerSecondSquared);
    kf_state_.state.bg = mean_gyr_;
    kf_state_.state.offset_T_L_I = params_.t_imu_lidar;
    kf_state_.state.offset_R_L_I = params_.r_imu_lidar;
    kf_->change_x(kf_state_.state);

    kf_state_.cov = PCov();
    kf_->change_P(kf_state_.cov);
    kf_state_.gyr = mean_gyr_;
  }
}

void ImuProcess::SyncWithLidar(rclcpp::Time* imu_start_time,
                               rclcpp::Time* imu_end_time) {
  imu_mutex_.lock();
  synced_imu_states_ = imu_states_;
  *imu_start_time = imu_start_time_;
  *imu_end_time = imu_end_time_;
  imu_mutex_.unlock();
}

void ImuProcess::GetTimeMatch(
    int* match_idx, const rclcpp::Time& match_time,
    const boost::circular_buffer<ImuState>& imu_states) {
  double time_diff;
  bool match_flag = false;

  time_diff = (match_time - imu_states.front().state.time).seconds();
  *match_idx = std::floor(time_diff * params_.rate);
  *match_idx = std::max(*match_idx, 0);
  *match_idx = std::min(*match_idx, static_cast<int>(imu_states.size()) - 1);

  while (!match_flag) {
    if (imu_states[*match_idx].state.time > match_time) {
      if (*match_idx == 0) {
        match_flag = true;
      } else if (imu_states[*match_idx - 1].state.time > match_time) {
        (*match_idx)--;
      } else {
        match_flag = true;
      }
    } else if (*match_idx == imu_states.size() - 1) {
      match_flag = true;
    } else {
      (*match_idx)++;
    }
  }
}

void ImuProcess::UndistortPointCloud(EllipseLioPointCloudPtr pc,
                                     KfState* kf_state,
                                     const rclcpp::Time& lidar_start_time,
                                     const rclcpp::Time& lidar_end_time,
                                     const CamProcessVec& cams) {
  int match_idx;
  Eigen::Isometry3d T_imu_lidar, T_world_imu_e;

  GetMatchingImages(lidar_start_time, lidar_end_time, cams, synced_imu_states_);
  GetTimeMatch(&match_idx, lidar_end_time, synced_imu_states_);

  *kf_state = synced_imu_states_[match_idx].state;

  T_imu_lidar.linear() =
      synced_imu_states_[match_idx].state.state.offset_R_L_I.toRotationMatrix();
  T_imu_lidar.translation() =
      synced_imu_states_[match_idx].state.state.offset_T_L_I;
  T_world_imu_e.linear() =
      synced_imu_states_[match_idx].state.state.rot.toRotationMatrix();
  T_world_imu_e.translation() = synced_imu_states_[match_idx].state.state.pos;

#pragma omp parallel for
  for (size_t i = 0; i < pc->points.size(); i++) {
    int head_idx, tail_idx;
    Eigen::Isometry3d T_world_imu_p, T_imu_e_imu_p;

    rclcpp::Time pt_time = rclcpp::Time(pc->points[i].time_secs,
                                        pc->points[i].time_nsecs, RCL_ROS_TIME);

    GetTimeMatch(&tail_idx, pt_time, synced_imu_states_);
    head_idx = std::max(tail_idx - 1, 0);

    M3D R_imu = synced_imu_states_[head_idx].state.state.rot.toRotationMatrix();
    V3D vel_imu = synced_imu_states_[head_idx].state.state.vel;
    V3D pos_imu = synced_imu_states_[head_idx].state.state.pos;
    V3D acc_avr = synced_imu_states_[tail_idx].acc_avr;
    V3D gyr_avr = synced_imu_states_[tail_idx].gyr_avr;

    double dt = (pt_time - synced_imu_states_[head_idx].state.time).seconds();

    T_world_imu_p.linear() = R_imu * Exp(gyr_avr, dt);
    T_world_imu_p.translation() =
        pos_imu + vel_imu * dt + 0.5 * acc_avr * dt * dt;
    T_imu_e_imu_p = T_world_imu_e.inverse() * T_world_imu_p;

    ColorisePoint(&pc->points[i], cams, T_world_imu_p, T_imu_lidar);

    pc->points[i].getVector3fMap() =
        (T_imu_lidar.inverse() * T_imu_e_imu_p * T_imu_lidar *
         pc->points[i].getVector3fMap().cast<double>())
            .cast<float>();
  }
}

void ImuProcess::GetMatchingImages(
    const rclcpp::Time& min_time, const rclcpp::Time& match_time,
    const CamProcessVec& cams,
    const boost::circular_buffer<ImuState>& imu_states) {
#pragma omp parallel for
  for (size_t i = 0; i < cams.size(); i++) {
    rclcpp::Time img_time;
    int head_idx, tail_idx;
    Eigen::Isometry3d T_world_img;

    cams[i]->GetMatchingImageTime(match_time, &img_time);
    if (!cams[i]->has_img_match_) continue;

    GetTimeMatch(&tail_idx, img_time, imu_states);
    head_idx = std::max(tail_idx - 1, 0);

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

void ImuProcess::ColorisePoint(EllipseLioPoint* pt, const CamProcessVec& cams,
                               const Eigen::Isometry3d& T_world_pt,
                               const Eigen::Isometry3d& T_imu_lidar) {
  int valid_num;
  Eigen::MatrixXi cam_cols;
  Eigen::Vector3i cam_col;
  Eigen::VectorXi cam_sum;

  pt->r = 0;
  pt->g = 0;
  pt->b = 0;
  pt->a = 0;
  pt->has_rgb = false;

  if (!cams.size()) return;
  cam_sum = Eigen::VectorXi::Zero(cams.size());
  cam_cols = Eigen::MatrixXi::Zero(cams.size(), 3);

#pragma omp parallel for
  for (size_t i = 0; i < cams.size(); i++) {
    Eigen::Vector3i pt_col;
    Eigen::Vector3d pt_img;

    if (!cams[i]->has_img_match_) continue;

    Eigen::Isometry3d& T_cam_lidar = cams[i]->T_cam_lidar_;
    Eigen::Isometry3d& T_world_img = cams[i]->T_world_img_;

    pt_col = Eigen::Vector3i::Zero();
    pt_img = T_cam_lidar * T_imu_lidar.inverse() * T_world_img.inverse() *
             T_world_pt * T_imu_lidar * pt->getVector3fMap().cast<double>();

    if (cams[i]->ColorPoint(&pt_img, &pt_col)) {
      cam_cols.row(i) = pt_col;
      cam_sum(i) = 1;
    }
  }

  cam_col = cam_cols.colwise().sum();
  valid_num = cam_sum.sum();

  if (valid_num) {
    cam_col /= valid_num;

    pt->a = 255;
    pt->r = cam_col(0);
    pt->g = cam_col(1);
    pt->b = cam_col(2);
    pt->has_rgb = true;
  }
}

void ImuProcess::UpdateStatesWithLidar(KfState* kf_state,
                                       const rclcpp::Time& lidar_end_time,
                                       double max_solve_time) {
  int match_idx;
  double solve_time;
  IkfomSPtr kf(new Ikfom());
  auto& clk = *node_->get_clock();

  imu_mutex_.lock();
  *kf = *kf_;
  imu_mutex_.unlock();

  kf->change_x(kf_state->state);
  kf->change_P(kf_state->cov);
  if (!kf->update_iterated_dyn_share_modified_R(kLidarPointCovariance,
                                                max_solve_time)) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), clk, 1000,
                          "Insufficient matches for iEKF update");
    return;
  }

  imu_mutex_.lock();

  *kf_ = *kf;
  GetTimeMatch(&match_idx, lidar_end_time, imu_states_);
  imu_states_[match_idx].state.state = kf_->get_x();
  imu_states_[match_idx].state.cov = kf_->get_P();

  *kf_state = imu_states_[match_idx].state;

  for (size_t i = match_idx + 1; i < imu_states_.size(); i++) {
    double dt;
    int head_idx, tail_idx;
    input_ikfom in;
    V3D gyr_avr, acc_avr;

    tail_idx = i;
    head_idx = i - 1;

    gyr_avr = 0.5 * (imu_states_[tail_idx].gyr + imu_states_[head_idx].gyr);
    acc_avr = 0.5 * (imu_states_[tail_idx].acc + imu_states_[head_idx].acc);
    acc_avr *= kGravityMetersPerSecondSquared / mean_acc_.norm();

    dt = (imu_states_[tail_idx].state.time - imu_states_[head_idx].state.time)
             .seconds();

    in.acc = acc_avr;
    in.gyro = gyr_avr;
    kf_->predict(dt, q_, in);

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
  kf_state_.gyr = imu_states_.back().gyr;

  SetKfState();
  imu_mutex_.unlock();
}

void ImuProcess::SetKfState() {
  pub_mutex_.lock();
  pub_kf_state_ = kf_state_;
  pub_mutex_.unlock();
}

void ImuProcess::GetKfState(KfState* kf_state) const {
  std::lock_guard<std::mutex> lock(pub_mutex_);
  *kf_state = pub_kf_state_;
}
