#include "lidar_processing.h"

LidarProcess::~LidarProcess() {}

LidarProcess::LidarProcess(LidarParams params, float map_resolution,
                           rclcpp::Node::SharedPtr node)
    : params_(params),
      node_(node),
      lidar_counter_(0),
      process_pc_(new EllipseLioPointCloud()),
      ellipselio_pc_(new EllipseLioPointCloud()),
      lidar_time_offset_(0, 0) {
  lidar_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions lidar_opt;
  lidar_opt.callback_group = lidar_callback_group_;

  lidar_has_data_ = false;
  lidar_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  lidar_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  num_bins_ = ceil(params_.max_range + 1);
  scan_res_ = kPi * (params_.vertical_fov / (params_.scan_lines - 1)) / 180.0;
  max_octree_res_ = 10;

  bin_pcs_i_ = Eigen::ArrayXi::Zero(num_bins_);
  bin_pcs_sizes_ = Eigen::ArrayXi::Zero(num_bins_);
  bin_sizes_ = std::vector<std::atomic<int>>(num_bins_);
  bin_octrees_ = std::vector<iOctree::Octree>(num_bins_);
  bin_idxs_ = std::vector<std::vector<int>>(num_bins_,
                                            std::vector<int>(kMaxScanPoints));
  bin_pcs_ = std::vector<EllipseLioPointCloud>(num_bins_);
  bin_min_times_ = std::vector<rclcpp::Time>(num_bins_);
  bin_max_times_ = std::vector<rclcpp::Time>(num_bins_);

  std::fill(bin_sizes_.begin(), bin_sizes_.end(), 0);

  process_pc_->resize(kMaxScanPoints);
  ellipselio_pc_->reserve(kMaxProcPoints);

  bucket_sizes_ = std::vector<int>(num_bins_, 1);
  cnt_neighbours_ = std::vector<int>(num_bins_, 1);
  scan_line_sep_ = std::vector<float>(num_bins_, scan_res_);
  min_neighbours_ = std::vector<int>(num_bins_, kMinNeighbours);
  max_neighbours_ = std::vector<int>(num_bins_, kMaxNeighbours);
  search_radii_ = std::vector<float>(num_bins_, kMaxSearchRes);
  match_radii_ = std::vector<float>(num_bins_, kMaxSearchRes);
  octree_resolutions_ = std::vector<float>(num_bins_, kMinSearchRes);

#pragma omp parallel for
  for (size_t i = 0; i < num_bins_; i++) {
    float octree_res, search_rad, bucket_size, scan_line_sep, match_rad;

    scan_line_sep = (i + 1) * scan_res_;
    scan_line_sep = floor(scan_line_sep * 100.0) / 100.0;
    scan_line_sep = fmax(scan_line_sep, kMinBinRes);

    octree_res = (i + 1) * scan_res_;
    octree_res = floor(octree_res * 100.0) / 100.0;
    octree_res = fmax(octree_res, kMinBinRes);

    match_rad = fmax(10.0 * octree_res, kMinSearchRes);
    search_rad = fmin(fmax(10.0 * octree_res, kMinSearchRes), kMaxSearchRes);

    bucket_size = kMaxNeighbours * pow(map_resolution, 2);
    bucket_size /= kPi * pow(search_rad, 2);
    bucket_size = fmin(ceil(bucket_size), kMinNeighbours);

    bucket_sizes_[i] = bucket_size;
    search_radii_[i] = search_rad;
    match_radii_[i] = match_rad;
    scan_line_sep_[i] = scan_line_sep;
    octree_resolutions_[i] = octree_res;

    bin_pcs_i_(i) = i;
    bin_pcs_[i].reserve(kMaxScanPoints);
    bin_octrees_[i].SetMaxNewPoints(kMaxScanPoints);
    bin_octrees_[i].SetMaxOctants(0.1 * kMaxScanPoints);
  }

  sub_pcl_pc_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      params.topic, rclcpp::SensorDataQoS(),
      std::bind(&LidarProcess::LidarCallback, this, std::placeholders::_1),
      lidar_opt);
}

void LidarProcess::LidarCallback(
    const sensor_msgs::msg::PointCloud2::UniquePtr msg_in) {
  sensor_msgs::msg::PointCloud2::SharedPtr msg(
      new sensor_msgs::msg::PointCloud2(*msg_in));

  lidar_counter_++;

  lidar_time_offset_ = rclcpp::Duration(0, 0);
  if (rclcpp::Time(msg->header.stamp) < lidar_end_time_) {
    rcl_duration_t time_offset;
    time_offset.nanoseconds =
        (lidar_end_time_ - rclcpp::Time(msg->header.stamp)).nanoseconds();
    lidar_time_offset_ = rclcpp::Duration(time_offset);
    RCLCPP_ERROR_STREAM_ONCE(node_->get_logger(),
                             "WARNING: Lidar time offset detected!");
  }

  lidar_mutex_.lock();
  Process(msg);
  lidar_mutex_.unlock();
}

void LidarProcess::Process(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  auto& clk = *node_->get_clock();

  switch (static_cast<LidType>(params_.type)) {
    case LidType::kLivox:
      PointCloudHandler<LivoxPoint>(msg);
      break;
    case LidType::kVelodyne:
      PointCloudHandler<VelodynePoint>(msg);
      break;
    case LidType::kOuster:
      PointCloudHandler<OusterPoint>(msg);
      break;
    case LidType::kHesai:
      PointCloudHandler<HesaiPoint>(msg);
      break;
    case LidType::kGazebo:
      PointCloudHandler<GazeboPoint>(msg);
      break;
  }

  int end_bin = num_bins_;
  int prev_start_bin = -1;
  while (start_bin_ > prev_start_bin && prev_start_bin < max_octree_res_) {
#pragma omp parallel for
    for (size_t i = 0; i < end_bin; i++) {
      std::vector<int> new_idxs, added_idxs;
      if (!bin_sizes_[i]) continue;

      bin_pcs_sizes_[i] = 0;
      bin_pcs_[i].clear();
      bin_octrees_[i].clear();

      bin_octrees_[i].SetBucketSize(1);
      bin_octrees_[i].SetMinExtent(octree_resolutions_[fmax(i, start_bin_)]);
      bin_octrees_[i].Update(*process_pc_, bin_sizes_[i], bin_idxs_[i],
                             added_idxs, new_idxs);

      if (!added_idxs.size()) continue;
      bin_pcs_[i] = EllipseLioPointCloud(*process_pc_, added_idxs);
      bin_pcs_sizes_[i] = bin_pcs_[i].size();
      SetMinMaxTime(i);

      bin_idxs_[i] = added_idxs;
      bin_sizes_[i] = bin_pcs_sizes_[i];
    }

    float mean_bin = 0, mean_num = 0;
    int proc_points = bin_pcs_sizes_.sum();
    for (size_t i = 0; i < num_bins_; i++) {
      if (mean_num > 0.5 * proc_points && bin_sizes_[i] < 0.01 * proc_points) {
        break;
      }
      mean_num += bin_sizes_[i];
      mean_bin += i * bin_sizes_[i];
    }

    prev_start_bin = start_bin_;
    mean_bin_ = floor(mean_bin / mean_num);
    start_bin_ = fmin(mean_bin_, max_octree_res_);
    end_bin = start_bin_;
  }

  ellipselio_pc_->clear();
  bool init_time = true;
  for (size_t i = 0; i < num_bins_; i++) {
    if (!bin_pcs_sizes_[i]) continue;
    if ((ellipselio_pc_->size() + bin_pcs_[i].size()) > kMaxProcPoints) break;

    *ellipselio_pc_ += bin_pcs_[i];

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
}

void LidarProcess::ClearBins() {
#pragma omp parallel for
  for (size_t i = 0; i < num_bins_; i++) {
    if (!bin_pcs_sizes_[i]) continue;
    bin_pcs_sizes_[i] = 0;
    bin_pcs_[i].clear();
    bin_octrees_[i].clear();
  }
}

void LidarProcess::ClearPointCloud() {
  lidar_mutex_.lock();
  ClearBins();
  lidar_has_data_ = false;
  lidar_mutex_.unlock();
}

bool LidarProcess::GetPointCloud(EllipseLioPointCloudPtr pc,
                                 rclcpp::Time* start_time,
                                 rclcpp::Time* end_time,
                                 Eigen::ArrayXi* bin_pcs_sizes, int* start_bin,
                                 int* mean_bin) {
  if (!lidar_has_data_) return false;

  lidar_mutex_.lock();
  *mean_bin = mean_bin_;
  *start_bin = start_bin_;
  if (pc->empty()) {
    *start_time = lidar_start_time_;
  }
  *end_time = lidar_end_time_;
  *pc += *ellipselio_pc_;
  *bin_pcs_sizes += bin_pcs_sizes_;
  ClearBins();
  lidar_has_data_ = false;
  lidar_mutex_.unlock();

  return true;
}

void LidarProcess::SetMinMaxTime(int bin_idx) {
  rclcpp::Time pt1_time, pt2_time;

  EllipseLioPoint& pt1 = bin_pcs_[bin_idx].points[0];
  pt1_time = rclcpp::Time(pt1.time_secs, pt1.time_nsecs, RCL_ROS_TIME);
  bin_min_times_[bin_idx] = pt1_time;
  bin_max_times_[bin_idx] = pt1_time;
  for (size_t i = 1; i < bin_pcs_[bin_idx].size(); i++) {
    EllipseLioPoint& pt2 = bin_pcs_[bin_idx].points[i];
    pt2_time = rclcpp::Time(pt2.time_secs, pt2.time_nsecs, RCL_ROS_TIME);

    bin_min_times_[bin_idx] = std::min(bin_min_times_[bin_idx], pt2_time);
    bin_max_times_[bin_idx] = std::max(bin_max_times_[bin_idx], pt2_time);
  }
}

void LidarProcess::SetPoint(const LivoxPoint& in_pt0, const LivoxPoint& in_pt,
                            EllipseLioPoint* out_pt, rclcpp::Time* point_time) {
  out_pt->intensity = in_pt.intensity;
  *point_time = rclcpp::Time(in_pt.timestamp, RCL_ROS_TIME);
}

void LidarProcess::SetPoint(const VelodynePoint& in_pt0,
                            const VelodynePoint& in_pt, EllipseLioPoint* out_pt,
                            rclcpp::Time* point_time) {
  out_pt->intensity = in_pt.intensity;
  if (in_pt.time < 0.0) {
    *point_time -= rclcpp::Duration(0, fabs(in_pt.time) * 1e9);
  } else if (in_pt.time < 1.0) {
    *point_time += rclcpp::Duration(0, in_pt.time * 1e9);
  } else {
    *point_time += rclcpp::Duration(0, (in_pt.time - in_pt0.time) * 1e3);
  }
}

void LidarProcess::SetPoint(const OusterPoint& in_pt0, const OusterPoint& in_pt,
                            EllipseLioPoint* out_pt, rclcpp::Time* point_time) {
  out_pt->intensity = in_pt.intensity;
  *point_time += rclcpp::Duration(0, in_pt.t);
}

void LidarProcess::SetPoint(const HesaiPoint& in_pt0, const HesaiPoint& in_pt,
                            EllipseLioPoint* out_pt, rclcpp::Time* point_time) {
  out_pt->intensity = in_pt.intensity;
  *point_time = rclcpp::Time(in_pt.timestamp * 1e9, RCL_ROS_TIME);
}

void LidarProcess::SetPoint(const GazeboPoint& in_pt0, const GazeboPoint& in_pt,
                            EllipseLioPoint* out_pt, rclcpp::Time* point_time) {
  out_pt->intensity = in_pt.intensity;
}

template <typename InPtType>
void LidarProcess::ConvertPoint(pcl::PointCloud<InPtType>& in_pc, int pt_idx,
                                rclcpp::Time& point_time) {
  InPtType& in_pt0 = in_pc.points[0];
  InPtType& in_pt = in_pc.points[pt_idx];
  EllipseLioPoint& out_pt = process_pc_->points[pt_idx];

  out_pt.x = in_pt.x;
  out_pt.y = in_pt.y;
  out_pt.z = in_pt.z;

  SetPoint(in_pt0, in_pt, &out_pt, &point_time);

  builtin_interfaces::msg::Time msg_time = point_time;
  out_pt.time_secs = msg_time.sec;
  out_pt.time_nsecs = msg_time.nanosec;
}

template <typename InPtType>
void LidarProcess::PointCloudHandler(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<InPtType> in_pc;
  pcl::fromROSMsg(*msg, in_pc);
  int num_points, proc_points;
  std::atomic<int> rej_points = 0;
  float mean_bin = 0, mean_num = 0;

  std::fill(bin_sizes_.begin(), bin_sizes_.end(), 0);
  num_points = fmin(in_pc.size(), kMaxScanPoints);

#pragma omp parallel for
  for (size_t i = 0; i < num_points; i++) {
    rclcpp::Time point_time = msg->header.stamp;
    point_time += lidar_time_offset_;
    ConvertPoint<InPtType>(in_pc, i, point_time);
    float range = sqrt(process_pc_->points[i].x * process_pc_->points[i].x +
                       process_pc_->points[i].y * process_pc_->points[i].y +
                       process_pc_->points[i].z * process_pc_->points[i].z);

    if (range < params_.min_range || range > params_.max_range ||
        std::isnan(range) || std::isinf(range)) {
      rej_points++;
      continue;
    }

    int bin_idx = floor(range);
    process_pc_->points[i].bin_idx = bin_idx;
    bin_idxs_[bin_idx][bin_sizes_[bin_idx]++] = i;
  }

  proc_points = num_points - rej_points.load();

  for (size_t i = 0; i < num_bins_; i++) {
    if (mean_num > 0.5 * proc_points && bin_sizes_[i] < 0.01 * proc_points) {
      break;
    }
    mean_num += bin_sizes_[i];
    mean_bin += i * bin_sizes_[i];
  }

  mean_bin_ = floor(mean_bin / mean_num);
  start_bin_ = fmin(mean_bin_, max_octree_res_);
}
