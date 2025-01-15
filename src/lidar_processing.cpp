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
  lidar_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  lidar_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  num_bins_ = ceil(params.max_range / params.bin_size);

  bin_pcs_sizes_ = std::vector<int>(num_bins_, 0);
  bin_sizes_ = std::vector<std::atomic<int>>(num_bins_);
  bin_octrees_ = std::vector<iOctree::Octree>(num_bins_);
  bin_idxs_ =
      std::vector<std::vector<int>>(num_bins_, std::vector<int>(200000));
  bin_pcs_ = std::vector<EllipseLivoPointCloud>(num_bins_);
  bin_min_times_ = std::vector<rclcpp::Time>(num_bins_);
  bin_max_times_ = std::vector<rclcpp::Time>(num_bins_);

  std::fill(bin_sizes_.begin(), bin_sizes_.end(), 0);

  bucket_sizes_ = std::vector<int>(num_bins_, 1);
  cnt_neighbours_ = std::vector<int>(num_bins_, 0);
  min_neighbours_ = std::vector<int>(num_bins_, MIN_NEIGHBOURS);
  max_neighbours_ = std::vector<int>(num_bins_, MAX_NEIGHBOURS);
  search_radii_ = std::vector<float>(num_bins_, params_.map_search_radius);
  octree_resolutions_ = std::vector<float>(num_bins_, params_.map_resolution);

#pragma omp parallel for
  for (size_t i = 0; i < num_bins_; i++) {
    float octree_res = (i + 1) * params_.bin_size * params_.downsample_factor;
    float search_radius = fmin(10.0 * octree_res, params_.map_search_radius);
    int bucket_size = ceil(MIN_NEIGHBOURS / pow(2, fmin(i, 10)));

    bucket_sizes_[i] = bucket_size;
    search_radii_[i] = search_radius;
    octree_resolutions_[i] = octree_res;
  }
}

void LidarProcess::LidarCallback(
    const sensor_msgs::msg::PointCloud2::UniquePtr msg_in) {
  sensor_msgs::msg::PointCloud2::SharedPtr msg(
      new sensor_msgs::msg::PointCloud2(*msg_in));

  if (rclcpp::Time(msg->header.stamp) < lidar_end_time_) {
    std::cerr << "Lidar time out of order" << std::endl;
    return;
  }

  double t1 = omp_get_wtime();
  Process(msg);
  double t2 = omp_get_wtime();
  std::cerr << "Lidar processing time: " << t2 - t1 << std::endl;
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

#pragma omp parallel for
  for (size_t i = 0; i < num_bins_; i++) {
    std::vector<int> new_idxs, added_idxs;
    if (!bin_sizes_[i]) continue;

    bin_octrees_[i].set_bucket_size(1);
    bin_octrees_[i].set_min_extent(octree_resolutions_[i]);
    bin_octrees_[i].initialize(*out_pc, bin_sizes_[i], bin_idxs_[i], added_idxs,
                               new_idxs);

    if (!added_idxs.size()) continue;
    bin_pcs_[i] += EllipseLivoPointCloud(*out_pc, added_idxs);
    bin_pcs_sizes_[i] = bin_pcs_[i].size();
    SetMinMaxTime(i);
  }

  lidar_mutex_.lock();

  ellipselivo_pc_->clear();
  bool init_time = true;
  for (size_t i = 0; i < num_bins_; i++) {
    if (!bin_pcs_sizes_[i]) continue;
    *ellipselivo_pc_ += bin_pcs_[i];

    if (init_time) {
      lidar_start_time_ = bin_min_times_[i];
      lidar_end_time_ = bin_max_times_[i];
      init_time = false;
    } else {
      lidar_start_time_ = std::min(lidar_start_time_, bin_min_times_[i]);
      lidar_end_time_ = std::max(lidar_end_time_, bin_max_times_[i]);
    }
  }
  lidar_has_data_ = true;

  lidar_mutex_.unlock();
}

void LidarProcess::ClearBins() {
#pragma omp parallel for
  for (size_t i = 0; i < num_bins_; i++) {
    if (!bin_pcs_sizes_[i]) continue;
    bin_pcs_sizes_[i] = 0;
    bin_pcs_[i].clear();
  }
}

void LidarProcess::ClearPointCloud() {
  lidar_mutex_.lock();
  ClearBins();
  lidar_has_data_ = false;
  lidar_mutex_.unlock();
}

void LidarProcess::GetPointCloud(EllipseLivoPointCloudPtr pc,
                                 rclcpp::Time &end_time,
                                 std::vector<int> &bin_pcs_sizes,
                                 int &start_bin) {
  lidar_mutex_.lock();
  *pc = *ellipselivo_pc_;
  start_bin = start_bin_;
  end_time = lidar_end_time_;
  bin_pcs_sizes = bin_pcs_sizes_;
  ClearBins();
  lidar_has_data_ = false;
  lidar_mutex_.unlock();
}

void LidarProcess::SetMinMaxTime(int bin_idx) {
  rclcpp::Time pt1_time, pt2_time;

  EllipseLivoPoint &pt1 = bin_pcs_[bin_idx].points[0];
  pt1_time = rclcpp::Time(pt1.time_secs, pt1.time_nsecs, RCL_ROS_TIME);
  bin_min_times_[bin_idx] = pt1_time;
  bin_max_times_[bin_idx] = pt1_time;
  for (size_t i = 1; i < bin_pcs_[bin_idx].size(); i++) {
    EllipseLivoPoint &pt2 = bin_pcs_[bin_idx].points[i];
    pt2_time = rclcpp::Time(pt2.time_secs, pt2.time_nsecs, RCL_ROS_TIME);

    bin_min_times_[bin_idx] = std::min(bin_min_times_[bin_idx], pt2_time);
    bin_max_times_[bin_idx] = std::max(bin_max_times_[bin_idx], pt2_time);
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

  Eigen::ArrayXf ranges(in_pc.size());
  Eigen::ArrayXf valid_range = Eigen::ArrayXf::Zero(in_pc.size());
  std::fill(bin_sizes_.begin(), bin_sizes_.end(), 0);

#pragma omp parallel for
  for (size_t i = 0; i < in_pc.size(); i++) {
    rclcpp::Time point_time = msg->header.stamp;
    ConvertPoint<InPtType>(in_pc.points[i], out_pc->points[i], point_time);
    float range = sqrt(out_pc->points[i].x * out_pc->points[i].x +
                       out_pc->points[i].y * out_pc->points[i].y +
                       out_pc->points[i].z * out_pc->points[i].z);

    ranges(i) = range;
    if (range < params_.min_range || range > params_.max_range) {
      continue;
    }
    valid_range(i) = 1;

    int bin_idx = floor(range / params_.bin_size);
    bin_idxs_[bin_idx][bin_sizes_[bin_idx]++] = i;

    out_pc->points[i].bin_idx = bin_idx;
  }

  mean_range_ = (ranges * valid_range).sum() / valid_range.sum();
  start_bin_ = round(mean_range_ / params_.bin_size);
}
