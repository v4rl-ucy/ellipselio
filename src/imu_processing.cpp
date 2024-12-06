#include <imu_processing.h>

ImuProcess::~ImuProcess() {}

ImuProcess::ImuProcess(KfFastlioSPtr kf, int imu_freq, std::string imu_topic,
                       rclcpp::Node::SharedPtr node)
    : b_first_frame_(true),
      imu_need_init_(true),
      imu_freq_(imu_freq),
      kf_(kf),
      node_(node),
      imu_states_(2 * imu_freq) {
  imu_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions imu_opt;
  imu_opt.callback_group = imu_callback_group_;

  sub_imu_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&ImuProcess::ImuCallback, this, std::placeholders::_1),
      imu_opt);

  imu_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  imu_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  Q = process_noise_cov();
  cov_acc = V3D(0.1, 0.1, 0.1);
  cov_gyr = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc = V3D(0.0001, 0.0001, 0.0001);
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;
}

void ImuProcess::ImuCallback(const sensor_msgs::msg::Imu::UniquePtr msg_in) {
  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

  if (rclcpp::Time(msg->header.stamp) < imu_end_time_) {
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
}

void ImuProcess::Reset() {
  init_iter_num = 1;
  imu_need_init_ = true;
  Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
  Q.block<3, 3>(3, 3).diagonal() = cov_acc;
  Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
  Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot) {
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}

void ImuProcess::set_gyr_cov(const V3D &gyr_cov) { cov_gyr = gyr_cov; }

void ImuProcess::set_acc_cov(const V3D &acc_cov) { cov_acc = acc_cov; }

void ImuProcess::set_gyr_bias_cov(const V3D &b_g) { cov_bias_gyr = b_g; }

void ImuProcess::set_acc_bias_cov(const V3D &b_a) { cov_bias_acc = b_a; }

void ImuProcess::InitImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/

  if (b_first_frame_) {
    Reset();
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

    mean_acc += (cur_acc - mean_acc) / (init_iter_num + 1);
    mean_gyr += (cur_gyr - mean_gyr) / (init_iter_num + 1);

    init_iter_num++;
  }

  if (init_iter_num > MAX_INI_COUNT) {
    imu_need_init_ = false;

    kf_state_.time = rclcpp::Time(msg->header.stamp);

    kf_state_.state = kf_->get_x();
    kf_state_.state.grav = S2(-mean_acc / mean_acc.norm() * G_m_s2);
    kf_state_.state.bg = mean_gyr;
    kf_state_.state.offset_T_L_I = Lidar_T_wrt_IMU;
    kf_state_.state.offset_R_L_I = Lidar_R_wrt_IMU;
    kf_->change_x(kf_state_.state);

    kf_state_.cov = kf_->get_P();
    kf_state_.cov.setIdentity();
    kf_state_.cov(6, 6) = kf_state_.cov(7, 7) = kf_state_.cov(8, 8) = 0.00001;
    kf_state_.cov(9, 9) = kf_state_.cov(10, 10) = kf_state_.cov(11, 11) =
        0.00001;
    kf_state_.cov(15, 15) = kf_state_.cov(16, 16) = kf_state_.cov(17, 17) =
        0.0001;
    kf_state_.cov(18, 18) = kf_state_.cov(19, 19) = kf_state_.cov(20, 20) =
        0.001;
    kf_state_.cov(21, 21) = kf_state_.cov(22, 22) = 0.00001;
    kf_->change_P(kf_state_.cov);
  }
}

void ImuProcess::GetTimeMatch(int &match_idx, rclcpp::Time &match_time,
                              boost::circular_buffer<ImuState> &imu_states) {
  double time_diff;
  bool match_flag = false;

  time_diff = (match_time - imu_states.front().state.time).seconds();
  match_idx = std::floor(time_diff * imu_freq_);
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

void ImuProcess::UndistortPointCloud(FastLioPointCloudPtr pc, KfState &kf_state,
                                     rclcpp::Time &lidar_end_time) {
  int match_idx;
  boost::circular_buffer<ImuState> imu_states;
  Eigen::Isometry3d T_imu_lidar, T_world_imu_e;

  imu_mutex_.lock();
  imu_states = imu_states_;
  imu_mutex_.unlock();

  GetTimeMatch(match_idx, lidar_end_time, imu_states);

  kf_state = imu_states[match_idx].state;

  T_imu_lidar.linear() =
      imu_states[match_idx].state.state.offset_R_L_I.toRotationMatrix();
  T_imu_lidar.translation() = imu_states[match_idx].state.state.offset_T_L_I;
  T_world_imu_e.linear() =
      imu_states[match_idx].state.state.rot.toRotationMatrix();
  T_world_imu_e.translation() = imu_states[match_idx].state.state.pos;

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
    pc->points[i].getVector3fMap() =
        (T_imu_lidar.inverse() * T_imu_e_imu_p * T_imu_lidar *
         pc->points[i].getVector3fMap().cast<double>())
            .cast<float>();
  }
}

void ImuProcess::UpdateStatesWithLidar(double &solve_H_time, KfState &kf_state,
                                       rclcpp::Time &lidar_end_time) {
  int match_idx;
  KfFastlioSPtr kf(new KfFastlio());

  imu_mutex_.lock();
  *kf = *kf_;
  imu_mutex_.unlock();

  kf->change_x(kf_state.state);
  kf->change_P(kf_state.cov);
  kf->update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);

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

void ImuProcess::GetKfState(KfState &kf_state) {
  imu_mutex_.lock();
  kf_state = kf_state_;
  imu_mutex_.unlock();
}
