#include <imu_processing.h>

ImuProcess::~ImuProcess() {}

ImuProcess::ImuProcess(KfFastlioSPtr kf, StateTimeSPtr kf_state, int imu_freq,
                       std::string imu_topic, rclcpp::Node::SharedPtr node)
    : b_first_frame_(true),
      imu_need_init_(true),
      start_timestamp_(-1),
      imu_freq_(imu_freq),
      kf_(kf),
      kf_state_(kf_state),
      node_(node),
      imu_buffer_(2 * imu_freq),
      imu_poses_(2 * imu_freq) {
  init_iter_num = 1;
  Q = process_noise_cov();
  cov_acc = V3D(0.1, 0.1, 0.1);
  cov_gyr = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc = V3D(0.0001, 0.0001, 0.0001);
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;

  imu_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions imu_opt;
  imu_opt.callback_group = imu_callback_group_;

  sub_imu_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&LaserMappingNode::ImuCallback, this, std::placeholders::_1),
      imu_opt);
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
  Pose6D imu_pose;
  V3D gyr_avr, acc_avr;

  if (imu_need_init_) {
    IMU_init(msg);
    return;
  }

  if (!imu_buffer_.size()) {
    acc_avr = mean_acc * G_m_s2 / mean_acc.norm();
    gyr_avr = mean_gyr;
  } else {
    gyr_avr = msg->angular_velocity + imu_buffer_.back()->angular_velocity;
    gyr_avr = gyr_avr / 2.0;
    acc_avr =
        msg->linear_acceleration + imu_buffer_.back()->linear_acceleration;
    acc_avr = acc_avr / 2.0;
    acc_avr = acc_avr * G_m_s2 / mean_acc.norm();
  }

  dt = (rclcpp::Time(msg->header.stamp) - kf_state_->time).seconds();

  in.acc = acc_avr;
  in.gyro = gyr_avr;
  kf_.predict(dt, Q, in);

  kf_state_->state = kf_->get_x();
  kf_state_->time = rclcpp::Time(msg->header.stamp);

  imu_pose.time = rclcpp::Time(msg->header.stamp);
  imu_pose.acc = kf_state_->state.rot * (acc_avr - kf_state_->state.ba);
  imu_pose.acc += kf_state_->state.grav.get_vect();
  imu_pose.gyr = gyr_avr - kf_state_->state.bg;
  imu_pose.vel = kf_state_->state.vel;
  imu_pose.pos = kf_state_->state.pos;
  imu_pose.rot = kf_state_->state.rot.toRotationMatrix();

  imu_buffer_.push_back(msg);
  imu_poses_.push_back(imu_pose);
  kf_states_.push_back(*kf_state_);
  imu_start_time_ = rclcpp::Time(imu_buffer_.front()->header.stamp);
  imu_end_time_ = rclcpp::Time(imu_buffer_.back()->header.stamp);
}

void ImuProcess::Reset() {
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  imu_need_init_ = true;
  start_timestamp_ = -1;
  init_iter_num = 1;
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

void ImuProcess::IMU_init(const sensor_msgs::msg::Imu::SharedPtr msg) {
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/

  V3D cur_acc, cur_gyr;

  if (b_first_frame_) {
    Reset();
    b_first_frame_ = false;
    const auto &imu_acc = msg->linear_acceleration;
    const auto &gyr_acc = msg->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
  } else {
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

    kf_state_->state = kf_->get_x();
    kf_state_->state.grav = S2(-mean_acc / mean_acc.norm() * G_m_s2);
    kf_state_->state.bg = mean_gyr;
    kf_state_->state.offset_T_L_I = Lidar_T_wrt_IMU;
    kf_state_->state.offset_R_L_I = Lidar_R_wrt_IMU;
    kf_->change_x(kf_state->state);

    kf_state_->time = rclcpp::Time(msg->header.stamp);

    KfFastlio::cov init_P = kf_->get_P();
    init_P.setIdentity();
    init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
    init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
    init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
    init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
    init_P(21, 21) = init_P(22, 22) = 0.00001;
    kf_->change_P(init_P);
  }
}

void ImuProcess::UndistortPcl(FastLioPointCloudPtr pc,
                              rclcpp::Time lidar_end_time) {
  /*** forward propagation at each imu point ***/
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  M3D R_imu;

  int match_idx;
  bool match_flag = false;
  double dt, match_time;

  Eigen::Vector3d P_i, P_dash;
  Eigen::Isometry3d T_imu_lidar, T_world_imu_p, T_world_imu_e, T_imu_e_imu_p;

  imu_mutex_.lock();

  match_time = (lidar_end_time - imu_start_time_).seconds();
  match_idx = std::floor(match_time * imu_freq_);

  while (!match_flag) {
    if (imu_poses_[match_idx].time > lidar_end_time) {
      match_flag = true;
    } else {
      match_idx--;
    }
  }

  T_imu_lidar.linear() = imu_state.offset_R_L_I.toRotationMatrix();
  T_imu_lidar.translation() = imu_state.offset_T_L_I;
  T_world_imu_e.linear() = imu_state.rot.toRotationMatrix();
  T_world_imu_e.translation() = imu_state.pos;

  /*** undistort each lidar point (backward propagation) ***/
  if (pcl_out.points.begin() == pcl_out.points.end()) return;
  auto it_pcl = pcl_out.points.end() - 1;
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--) {
    auto head = it_kp - 1;
    auto tail = it_kp;
    R_imu << MAT_FROM_ARRAY(head->rot);
    vel_imu << VEC_FROM_ARRAY(head->vel);
    pos_imu << VEC_FROM_ARRAY(head->pos);
    acc_imu << VEC_FROM_ARRAY(tail->acc);
    angvel_avr << VEC_FROM_ARRAY(tail->gyr);

    for (; it_pcl->offset_time / double(1000) > head->offset_time; it_pcl--) {
      dt = it_pcl->offset_time / double(1000) - head->offset_time;

      P_i << it_pcl->x, it_pcl->y, it_pcl->z;
      T_world_imu_p.linear() = R_imu * Exp(angvel_avr, dt);
      T_world_imu_p.translation() =
          pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
      T_imu_e_imu_p = T_world_imu_e.inverse() * T_world_imu_p;
      P_dash = T_imu_lidar.inverse() * T_imu_e_imu_p * T_imu_lidar * P_i;

      it_pcl->x = P_dash(0);
      it_pcl->y = P_dash(1);
      it_pcl->z = P_dash(2);

      if (it_pcl == pcl_out.points.begin()) break;
    }
  }
  imu_mutex_.unlock();
}
