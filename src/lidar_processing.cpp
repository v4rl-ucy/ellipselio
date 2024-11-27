#include <lidar_processing.h>

LidarProcess::~LidarProcess() {}

LidarProcess::LidarProcess(int lidar_type, float min_range, float max_range,
                           std::string lidar_topic,
                           rclcpp::Node::SharedPtr node)
    : node_(node), fastlio_pc_(new FastLioPointCloud()) {
  min_range_ = min_range;
  max_range_ = max_range;
  lidar_type_ = lidar_type;

  lidar_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions lidar_opt;
  lidar_opt.callback_group = lidar_callback_group_;

  sub_pcl_pc_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic, rclcpp::SensorDataQoS(),
      std::bind(&LidarProcess::LidarCallback, this, std::placeholders::_1),
      lidar_opt);
}

void LidarProcess::LidarCallback(
    const sensor_msgs::msg::PointCloud2::UniquePtr msg) {
  if (rclcpp::Time(msg->header.stamp) < lidar_end_time_) {
    return;
  }

  Process(msg);
}

void LidarProcess::Process(const sensor_msgs::msg::PointCloud2::UniquePtr msg) {
  FastLioPointCloudPtr new_pc(new FastLioPointCloud());

  switch (lidar_type) {
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

  ioctree.set_bucket_size(1);
  ioctree.set_min_extent(scan_min_extent_);
  ioctree.initialize(new_pc, added_idxs, new_idxs);
  *fastlio_pc_ += FastLioPointCloud(*new_pc, added_idxs);

  int start_secs = fastlio_pc_->points.front().time_secs;
  int start_nsecs = fastlio_pc_->points.front().time_nsecs;
  int end_secs = fastlio_pc_->points.back().time_secs;
  int end_nsecs = fastlio_pc_->points.back().time_nsecs;

  lidar_start_time_ = rclcpp::Time(start_secs, start_nsecs, RCL_ROS_TIME);
  lidar_end_time_ = rclcpp::Time(end_secs, end_nsecs, RCL_ROS_TIME);
}

void LidarProcess::OusterHandler(
    const sensor_msgs::msg::PointCloud2::UniquePtr msg,
    FastLioPointCloudPtr new_pc) {
  pcl::PointCloud<ouster_point> msg_pc;
  pcl::fromROSMsg(*msg, msg_pc);

  new_pc->reserve(msg_pc.size());

  mean_range = 0.0;
  for (int i = 0; i < msg_pc.points.size(); i++) {
    double range = msg_pc.points[i].x * msg_pc.points[i].x +
                   msg_pc.points[i].y * msg_pc.points[i].y +
                   msg_pc.points[i].z * msg_pc.points[i].z;

    mean_range += sqrt(range);
  }
  mean_range /= msg_pc.points.size();

  rclcpp::Time timestamp = msg->header.stamp;
  double dyn_range_min = fmin(min_range_, 0.1 * mean_range);
  double dyn_range_max = fmin(100, 10 * mean_range);

  for (int i = 0; i < msg_pc.points.size(); i++) {
    double range = msg_pc.points[i].x * msg_pc.points[i].x +
                   msg_pc.points[i].y * msg_pc.points[i].y +
                   msg_pc.points[i].z * msg_pc.points[i].z;

    if (sqrt(range) < dyn_range_min) continue;
    if (sqrt(range) > dyn_range_max) continue;

    rclcpp::Time point_time = timestamp;
    point_time += rclcpp::Duration(0, msg_pc.points[i].t);
    builtin_interfaces::msg::Time msg_time = point_time;

    FastLioPoint added_pt;
    added_pt.x = msg_pc.points[i].x;
    added_pt.y = msg_pc.points[i].y;
    added_pt.z = msg_pc.points[i].z;
    added_pt.intensity = msg_pc.points[i].intensity;
    added_pt.time_secs = msg_time.sec;
    added_pt.time_nsecs = msg_time.nsec;

    new_pc->push_back(std::move(added_pt));
  }
}

void LidarProcess::VelodyneHandler(
    const sensor_msgs::msg::PointCloud2::UniquePtr msg,
    FastLioPointCloudPtr new_pc) {
  pcl::PointCloud<velodyne_point> msg_pc;
  pcl::fromROSMsg(*msg, msg_pc);

  new_pc->reserve(msg_pc.size());

  mean_range = 0.0;
  for (int i = 0; i < msg_pc.points.size(); i++) {
    double range = msg_pc.points[i].x * msg_pc.points[i].x +
                   msg_pc.points[i].y * msg_pc.points[i].y +
                   msg_pc.points[i].z * msg_pc.points[i].z;

    mean_range += sqrt(range);
  }
  mean_range /= msg_pc.points.size();

  rclcpp::Time timestamp = msg->header.stamp;
  double dyn_range_min = fmin(min_range_, 0.1 * mean_range);
  double dyn_range_max = fmin(100, 10 * mean_range);

  for (int i = 0; i < msg_pc.points.size(); i++) {
    double range = msg_pc.points[i].x * msg_pc.points[i].x +
                   msg_pc.points[i].y * msg_pc.points[i].y +
                   msg_pc.points[i].z * msg_pc.points[i].z;

    if (sqrt(range) < dyn_range_min) continue;
    if (sqrt(range) > dyn_range_max) continue;

    rclcpp::Time point_time = timestamp;
    point_time += rclcpp::Duration(0, msg_pc.points[i].time * 1e3);
    builtin_interfaces::msg::Time msg_time = point_time;

    FastLioPoint added_pt;
    added_pt.x = msg_pc.points[i].x;
    added_pt.y = msg_pc.points[i].y;
    added_pt.z = msg_pc.points[i].z;
    added_pt.intensity = msg_pc.points[i].intensity;
    added_pt.time_secs = msg_time.sec;
    added_pt.time_nsecs = msg_time.nsec;

    new_pc->push_back(std::move(added_pt));
  }
}

void LidarProcess::LivoxHandler(
    const sensor_msgs::msg::PointCloud2::UniquePtr msg,
    FastLioPointCloudPtr new_pc) {
  pcl::PointCloud<livox_point> msg_pc;
  pcl::fromROSMsg(*msg, msg_pc);

  new_pc->reserve(msg_pc.size());

  mean_range = 0.0;
  for (int i = 0; i < msg_pc.points.size(); i++) {
    double range = msg_pc.points[i].x * msg_pc.points[i].x +
                   msg_pc.points[i].y * msg_pc.points[i].y +
                   msg_pc.points[i].z * msg_pc.points[i].z;

    mean_range += sqrt(range);
  }
  mean_range /= msg_pc.points.size();

  rclcpp::Time timestamp = msg->header.stamp;
  double dyn_range_min = fmin(min_range_, 0.1 * mean_range);
  double dyn_range_max = fmin(100, 10 * mean_range);

  for (int i = 0; i < msg_pc.points.size(); i++) {
    double range = msg_pc.points[i].x * msg_pc.points[i].x +
                   msg_pc.points[i].y * msg_pc.points[i].y +
                   msg_pc.points[i].z * msg_pc.points[i].z;

    if (sqrt(range) < dyn_range_min) continue;
    if (sqrt(range) > dyn_range_max) continue;

    rclcpp::Time point_time = timestamp;
    point_time += rclcpp::Duration(0, msg_pc.points[i].offset_time);
    builtin_interfaces::msg::Time msg_time = point_time;

    FastLioPoint added_pt;
    added_pt.x = msg_pc.points[i].x;
    added_pt.y = msg_pc.points[i].y;
    added_pt.z = msg_pc.points[i].z;
    added_pt.intensity = msg_pc.points[i].reflectivity;
    added_pt.time_secs = msg_time.sec;
    added_pt.time_nsecs = msg_time.nsec;

    new_pc->push_back(std::move(added_pt));
  }
}

void LidarProcess::HesaiHandler(
    const sensor_msgs::msg::PointCloud2::UniquePtr msg,
    FastLioPointCloudPtr new_pc) {
  pcl::PointCloud<hesai_point> msg_pc;
  pcl::fromROSMsg(*msg, msg_pc);

  new_pc->reserve(msg_pc.size());

  mean_range = 0.0;
  for (int i = 0; i < msg_pc.points.size(); i++) {
    double range = msg_pc.points[i].x * msg_pc.points[i].x +
                   msg_pc.points[i].y * msg_pc.points[i].y +
                   msg_pc.points[i].z * msg_pc.points[i].z;

    mean_range += sqrt(range);
  }
  mean_range /= msg_pc.points.size();

  double dyn_range_min = fmin(min_range_, 0.1 * mean_range);
  double dyn_range_max = fmin(100, 10 * mean_range);

  for (int i = 0; i < msg_pc.points.size(); i++) {
    double range = msg_pc.points[i].x * msg_pc.points[i].x +
                   msg_pc.points[i].y * msg_pc.points[i].y +
                   msg_pc.points[i].z * msg_pc.points[i].z;

    if (sqrt(range) < dyn_range_min) continue;
    if (sqrt(range) > dyn_range_max) continue;

    int64_t time_nsecs = msg_pc.points[i].timestamp * 1e9;
    rclcpp::Time point_time = rclcpp::Time(time_nsecs, RCL_ROS_TIME);
    builtin_interfaces::msg::Time msg_time = point_time;

    FastLioPoint added_pt;
    added_pt.x = msg_pc.points[i].x;
    added_pt.y = msg_pc.points[i].y;
    added_pt.z = msg_pc.points[i].z;
    added_pt.intensity = msg_pc.points[i].intensity;
    added_pt.time_secs = msg_time.sec;
    added_pt.time_nsecs = msg_time.nsec;

    new_pc->push_back(std::move(added_pt));
  }
}