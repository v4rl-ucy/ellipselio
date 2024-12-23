#include <lidar_processing.h>

LidarProcess::~LidarProcess() {}

LidarProcess::LidarProcess(LidarParams params, rclcpp::Node::SharedPtr node)
    : params_(params),
      node_(node),
      ellipselivo_pc_(new EllipseLivoPointCloud()) {
  lidar_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions lidar_opt;
  lidar_opt.callback_group = lidar_callback_group_;

  sub_pcl_pc_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      params.topic, rclcpp::SensorDataQoS(),
      std::bind(&LidarProcess::LidarCallback, this, std::placeholders::_1),
      lidar_opt);

  lidar_has_data_ = false;
  upd_lidar_has_data_ = false;
  lidar_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  lidar_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  upd_lidar_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  upd_lidar_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  int num_bins = ceil((params.max_range - params.min_range) / params.bin_size);

  bin_size_ = std::vector<std::atomic<int>>(num_bins);
  bin_octrees_ = std::vector<iOctree::Octree>(num_bins);
  bin_idxs_ = std::vector<std::vector<int>>(num_bins, std::vector<int>(200000));
  new_idxs_ = std::vector<std::vector<int>>(num_bins, std::vector<int>());
  added_idxs_ = std::vector<std::vector<int>>(num_bins, std::vector<int>());

  std::fill(bin_size_.begin(), bin_size_.end(), 0);
}

void LidarProcess::LidarCallback(
    const sensor_msgs::msg::PointCloud2::UniquePtr msg_in) {
  sensor_msgs::msg::PointCloud2::SharedPtr msg(
      new sensor_msgs::msg::PointCloud2(*msg_in));

  if (rclcpp::Time(msg->header.stamp) < lidar_end_time_) return;
  Process(msg);
}

void LidarProcess::Process(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  EllipseLivoPointCloudPtr out_pc(new EllipseLivoPointCloud());

  switch (params_.type) {
    case LIVOX:
      PointCloudHandler<LivoxPoint>(msg, out_pc);
      break;
    case VELODYNE:
      PointCloudHandler<VelodynePoint>(msg, out_pc);
      break;
    case OUSTER:
      PointCloudHandler<OusterPoint>(msg, out_pc);
      break;
    case HESAI:
      PointCloudHandler<HesaiPoint>(msg, out_pc);
      break;
  }

  upd_lidar_has_data_ = lidar_has_data_;

#pragma omp parallel for
  for (size_t i = 0; i < bin_size_.size(); i++) {
    float oct_res = (i + 1) * params_.bin_size * params_.downsample_factor;
    if (bin_size_[i] == 0) {
      added_idxs_[i].clear();
      new_idxs_[i].clear();
      continue;
    }

    bin_octrees_[i].set_bucket_size(1);
    bin_octrees_[i].set_min_extent(oct_res);
    bin_octrees_[i].initialize(*out_pc, bin_size_[i], bin_idxs_[i],
                               added_idxs_[i], new_idxs_[i]);
    bin_size_[i] = 0;
  }

  lidar_mutex_.lock();
  for (size_t i = 0; i < bin_size_.size(); i++) {
    if (!added_idxs_[i].size()) continue;

    *ellipselivo_pc_ += EllipseLivoPointCloud(*out_pc, added_idxs_[i]);
    for (size_t j = 0; j < added_idxs_[i].size(); j++) {
      SetMinMaxTime(out_pc->points[added_idxs_[i][j]]);
    }
  }

  lidar_start_time_ = upd_lidar_start_time_;
  lidar_end_time_ = upd_lidar_end_time_;
  lidar_has_data_ = upd_lidar_has_data_;
  lidar_mutex_.unlock();
}

void LidarProcess::ClearPointCloud() {
  lidar_mutex_.lock();
  ellipselivo_pc_->clear();
  lidar_has_data_ = false;
  lidar_mutex_.unlock();
}

void LidarProcess::GetPointCloud(EllipseLivoPointCloudPtr pc,
                                 rclcpp::Time &end_time) {
  lidar_mutex_.lock();
  *pc = *ellipselivo_pc_;
  end_time = lidar_end_time_;
  ellipselivo_pc_->clear();
  lidar_has_data_ = false;
  lidar_mutex_.unlock();
}

void LidarProcess::SetMinMaxTime(EllipseLivoPoint &pt) {
  rclcpp::Time pt_time;
  pt_time = rclcpp::Time(pt.time_secs, pt.time_nsecs, RCL_ROS_TIME);

  if (upd_lidar_has_data_) {
    upd_lidar_start_time_ = std::min(upd_lidar_start_time_, pt_time);
    upd_lidar_end_time_ = std::max(upd_lidar_end_time_, pt_time);
  } else {
    upd_lidar_start_time_ = pt_time;
    upd_lidar_end_time_ = pt_time;
    upd_lidar_has_data_ = true;
  }
}

void LidarProcess::SetPoint(LivoxPoint &in_pt, EllipseLivoPoint &out_pt,
                            rclcpp::Time &point_time) {
  out_pt.intensity = in_pt.reflectivity;
  point_time += rclcpp::Duration(0, in_pt.offset_time);
}

void LidarProcess::SetPoint(VelodynePoint &in_pt, EllipseLivoPoint &out_pt,
                            rclcpp::Time &point_time) {
  out_pt.intensity = in_pt.intensity;
  point_time += rclcpp::Duration(0, in_pt.time * 1e3);
}

void LidarProcess::SetPoint(OusterPoint &in_pt, EllipseLivoPoint &out_pt,
                            rclcpp::Time &point_time) {
  out_pt.intensity = in_pt.intensity;
  point_time += rclcpp::Duration(0, in_pt.t);
}

void LidarProcess::SetPoint(HesaiPoint &in_pt, EllipseLivoPoint &out_pt,
                            rclcpp::Time &point_time) {
  out_pt.intensity = in_pt.intensity;
  point_time = rclcpp::Time(in_pt.timestamp * 1e9, RCL_ROS_TIME);
}

template <typename InPtType>
void LidarProcess::ConvertPoint(InPtType &in_pt, EllipseLivoPoint &out_pt,
                                rclcpp::Time &point_time) {
  out_pt.x = in_pt.x;
  out_pt.y = in_pt.y;
  out_pt.z = in_pt.z;

  SetPoint(in_pt, out_pt, point_time);

  builtin_interfaces::msg::Time msg_time = point_time;
  out_pt.time_secs = msg_time.sec;
  out_pt.time_nsecs = msg_time.nanosec;
}

template <typename InPtType>
void LidarProcess::PointCloudHandler(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    EllipseLivoPointCloudPtr out_pc) {
  pcl::PointCloud<InPtType> in_pc;
  pcl::fromROSMsg(*msg, in_pc);

  out_pc->resize(in_pc.size());

#pragma omp parallel for
  for (size_t i = 0; i < in_pc.size(); i++) {
    rclcpp::Time point_time = msg->header.stamp;
    ConvertPoint<InPtType>(in_pc.points[i], out_pc->points[i], point_time);
    double range = sqrt(out_pc->points[i].x * out_pc->points[i].x +
                        out_pc->points[i].y * out_pc->points[i].y +
                        out_pc->points[i].z * out_pc->points[i].z);
    if (range < params_.min_range || range > params_.max_range) {
      continue;
    }
    int bin_idx = floor((range - params_.min_range) / params_.bin_size);
    bin_idxs_[bin_idx][bin_size_[bin_idx]++] = i;
  }
}
