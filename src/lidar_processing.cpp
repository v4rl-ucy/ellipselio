#include <lidar_processing.h>

LidarProcess::~LidarProcess() {}

LidarProcess::LidarProcess(int lidar_type, float min_range, float max_range,
                           std::string lidar_topic,
                           rclcpp::Node::SharedPtr node)
    : node_(node), fastlio_pc_(new FastLioPointCloud()) {
  min_range_ = min_range;
  max_range_ = max_range;
  lidar_type_ = lidar_type;

  lidar_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions lidar_opt;
  lidar_opt.callback_group = lidar_callback_group_;

  sub_pcl_pc_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic, rclcpp::SensorDataQoS(),
      std::bind(&LidarProcess::LidarCallback, this, std::placeholders::_1),
      lidar_opt);

  lidar_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  lidar_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  new_lidar_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  new_lidar_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void LidarProcess::LidarCallback(
    const sensor_msgs::msg::PointCloud2::UniquePtr msg_in) {
  sensor_msgs::msg::PointCloud2::SharedPtr msg(
      new sensor_msgs::msg::PointCloud2(*msg_in));

  if (rclcpp::Time(msg->header.stamp) < lidar_end_time_) {
    return;
  }

  Process(msg);
}

void LidarProcess::Process(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  FastLioPointCloudPtr new_pc(new FastLioPointCloud());

  switch (lidar_type_) {
    case LIVOX:
      LivoxHandler(msg, new_pc);
      break;
    case VELODYNE:
      VelodyneHandler(msg, new_pc);
      break;
    case OUSTER:
      OusterHandler(msg, new_pc);
      break;
    case HESAI:
      HesaiHandler(msg, new_pc);
      break;
  }

  std::vector<int> added_idxs, new_idxs;
  scan_min_extent_ = 0.02 * mean_range_;

  ioctree_.set_bucket_size(1);
  ioctree_.set_min_extent(scan_min_extent_);
  ioctree_.initialize(*new_pc, added_idxs, new_idxs);

  lidar_mutex_.lock();
  *fastlio_pc_ += FastLioPointCloud(*new_pc, added_idxs);

  lidar_start_time_ = new_lidar_start_time_;
  lidar_end_time_ = new_lidar_end_time_;
  lidar_mutex_.unlock();
}

void LidarProcess::ClearPointCloud() {
  lidar_mutex_.lock();
  fastlio_pc_->clear();
  lidar_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  lidar_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  new_lidar_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  new_lidar_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  lidar_mutex_.unlock();
}

void LidarProcess::GetPointCloud(FastLioPointCloudPtr pc,
                                 rclcpp::Time &end_time) {
  lidar_mutex_.lock();
  *pc = *fastlio_pc_;
  end_time = lidar_end_time_;
  fastlio_pc_->clear();
  lidar_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  lidar_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  new_lidar_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  new_lidar_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  lidar_mutex_.unlock();
}

void LidarProcess::SetMinMaxTime(rclcpp::Time &point_time) {
  if (new_lidar_start_time_ == rclcpp::Time(0, 0, RCL_ROS_TIME)) {
    new_lidar_start_time_ = point_time;
  }
  if (new_lidar_end_time_ == rclcpp::Time(0, 0, RCL_ROS_TIME)) {
    new_lidar_end_time_ = point_time;
  }

  new_lidar_start_time_ = std::min(new_lidar_start_time_, point_time);
  new_lidar_end_time_ = std::max(new_lidar_end_time_, point_time);
}

void LidarProcess::OusterHandler(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    FastLioPointCloudPtr new_pc) {
  pcl::PointCloud<ouster_point> msg_pc;
  pcl::fromROSMsg(*msg, msg_pc);

  new_pc->reserve(msg_pc.size());

  mean_range_ = 0.0;
  for (int i = 0; i < msg_pc.points.size(); i++) {
    double range = msg_pc.points[i].x * msg_pc.points[i].x +
                   msg_pc.points[i].y * msg_pc.points[i].y +
                   msg_pc.points[i].z * msg_pc.points[i].z;

    mean_range_ += sqrt(range);
  }
  mean_range_ /= msg_pc.points.size();

  rclcpp::Time timestamp = msg->header.stamp;
  double dyn_range_min = fmin(min_range_, 0.1 * mean_range_);
  double dyn_range_max = fmin(100, 10 * mean_range_);

  for (int i = 0; i < msg_pc.points.size(); i++) {
    double range = msg_pc.points[i].x * msg_pc.points[i].x +
                   msg_pc.points[i].y * msg_pc.points[i].y +
                   msg_pc.points[i].z * msg_pc.points[i].z;

    if (sqrt(range) < dyn_range_min) continue;
    if (sqrt(range) > dyn_range_max) continue;

    rclcpp::Time point_time = timestamp;
    point_time += rclcpp::Duration(0, msg_pc.points[i].t);
    builtin_interfaces::msg::Time msg_time = point_time;
    SetMinMaxTime(point_time);

    FastLioPoint added_pt;
    added_pt.x = msg_pc.points[i].x;
    added_pt.y = msg_pc.points[i].y;
    added_pt.z = msg_pc.points[i].z;
    added_pt.intensity = msg_pc.points[i].intensity;
    added_pt.time_secs = msg_time.sec;
    added_pt.time_nsecs = msg_time.nanosec;

    new_pc->push_back(std::move(added_pt));
  }
}

void LidarProcess::VelodyneHandler(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    FastLioPointCloudPtr new_pc) {
  pcl::PointCloud<velodyne_point> msg_pc;
  pcl::fromROSMsg(*msg, msg_pc);

  new_pc->reserve(msg_pc.size());

  mean_range_ = 0.0;
  for (int i = 0; i < msg_pc.points.size(); i++) {
    double range = msg_pc.points[i].x * msg_pc.points[i].x +
                   msg_pc.points[i].y * msg_pc.points[i].y +
                   msg_pc.points[i].z * msg_pc.points[i].z;

    mean_range_ += sqrt(range);
  }
  mean_range_ /= msg_pc.points.size();

  rclcpp::Time timestamp = msg->header.stamp;
  double dyn_range_min = fmin(min_range_, 0.1 * mean_range_);
  double dyn_range_max = fmin(100, 10 * mean_range_);

  for (int i = 0; i < msg_pc.points.size(); i++) {
    double range = msg_pc.points[i].x * msg_pc.points[i].x +
                   msg_pc.points[i].y * msg_pc.points[i].y +
                   msg_pc.points[i].z * msg_pc.points[i].z;

    if (sqrt(range) < dyn_range_min) continue;
    if (sqrt(range) > dyn_range_max) continue;

    rclcpp::Time point_time = timestamp;
    point_time += rclcpp::Duration(0, msg_pc.points[i].time * 1e3);
    builtin_interfaces::msg::Time msg_time = point_time;
    SetMinMaxTime(point_time);

    FastLioPoint added_pt;
    added_pt.x = msg_pc.points[i].x;
    added_pt.y = msg_pc.points[i].y;
    added_pt.z = msg_pc.points[i].z;
    added_pt.intensity = msg_pc.points[i].intensity;
    added_pt.time_secs = msg_time.sec;
    added_pt.time_nsecs = msg_time.nanosec;

    new_pc->push_back(std::move(added_pt));
  }
}

void LidarProcess::LivoxHandler(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    FastLioPointCloudPtr new_pc) {
  pcl::PointCloud<livox_point> msg_pc;
  pcl::fromROSMsg(*msg, msg_pc);

  new_pc->reserve(msg_pc.size());

  mean_range_ = 0.0;
  for (int i = 0; i < msg_pc.points.size(); i++) {
    double range = msg_pc.points[i].x * msg_pc.points[i].x +
                   msg_pc.points[i].y * msg_pc.points[i].y +
                   msg_pc.points[i].z * msg_pc.points[i].z;

    mean_range_ += sqrt(range);
  }
  mean_range_ /= msg_pc.points.size();

  rclcpp::Time timestamp = msg->header.stamp;
  double dyn_range_min = fmin(min_range_, 0.1 * mean_range_);
  double dyn_range_max = fmin(100, 10 * mean_range_);

  for (int i = 0; i < msg_pc.points.size(); i++) {
    double range = msg_pc.points[i].x * msg_pc.points[i].x +
                   msg_pc.points[i].y * msg_pc.points[i].y +
                   msg_pc.points[i].z * msg_pc.points[i].z;

    if (sqrt(range) < dyn_range_min) continue;
    if (sqrt(range) > dyn_range_max) continue;

    rclcpp::Time point_time = timestamp;
    point_time += rclcpp::Duration(0, msg_pc.points[i].offset_time);
    builtin_interfaces::msg::Time msg_time = point_time;
    SetMinMaxTime(point_time);

    FastLioPoint added_pt;
    added_pt.x = msg_pc.points[i].x;
    added_pt.y = msg_pc.points[i].y;
    added_pt.z = msg_pc.points[i].z;
    added_pt.intensity = msg_pc.points[i].reflectivity;
    added_pt.time_secs = msg_time.sec;
    added_pt.time_nsecs = msg_time.nanosec;

    new_pc->push_back(std::move(added_pt));
  }
}

void LidarProcess::HesaiHandler(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    FastLioPointCloudPtr new_pc) {
  pcl::PointCloud<hesai_point> msg_pc;
  pcl::fromROSMsg(*msg, msg_pc);

  new_pc->reserve(msg_pc.size());

  mean_range_ = 0.0;
  for (int i = 0; i < msg_pc.points.size(); i++) {
    double range = msg_pc.points[i].x * msg_pc.points[i].x +
                   msg_pc.points[i].y * msg_pc.points[i].y +
                   msg_pc.points[i].z * msg_pc.points[i].z;

    mean_range_ += sqrt(range);
  }
  mean_range_ /= msg_pc.points.size();

  double dyn_range_min = fmin(min_range_, 0.1 * mean_range_);
  double dyn_range_max = fmin(100, 10 * mean_range_);

  for (int i = 0; i < msg_pc.points.size(); i++) {
    double range = msg_pc.points[i].x * msg_pc.points[i].x +
                   msg_pc.points[i].y * msg_pc.points[i].y +
                   msg_pc.points[i].z * msg_pc.points[i].z;

    if (sqrt(range) < dyn_range_min) continue;
    if (sqrt(range) > dyn_range_max) continue;

    int64_t time_nsecs = msg_pc.points[i].timestamp * 1e9;
    rclcpp::Time point_time = rclcpp::Time(time_nsecs, RCL_ROS_TIME);
    builtin_interfaces::msg::Time msg_time = point_time;
    SetMinMaxTime(point_time);

    FastLioPoint added_pt;
    added_pt.x = msg_pc.points[i].x;
    added_pt.y = msg_pc.points[i].y;
    added_pt.z = msg_pc.points[i].z;
    added_pt.intensity = msg_pc.points[i].intensity;
    added_pt.time_secs = msg_time.sec;
    added_pt.time_nsecs = msg_time.nanosec;

    new_pc->push_back(std::move(added_pt));
  }
}