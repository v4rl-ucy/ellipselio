#include <lidar_processing.h>

LidarProcess::~LidarProcess() {}

LidarProcess::LidarProcess(int lidar_type, int time_unit, float min_range,
                           float max_range) {
  min_range_ = min_range;
  max_range_ = max_range;

  lidar_type_ = lidar_type;

  switch (time_unit) {
    case SEC:
      time_unit_scale_ = 1.e3f;
      break;
    case MS:
      time_unit_scale_ = 1.f;
      break;
    case US:
      time_unit_scale_ = 1.e-3f;
      break;
    case NS:
      time_unit_scale_ = 1.e-6f;
      break;
  }
}

void LidarProcess::LidarCallback(
    const sensor_msgs::msg::PointCloud2::UniquePtr msg) {
  scan_count++;
  double cur_time = get_time_sec(msg->header.stamp);
  double preprocess_start_time = omp_get_wtime();
  if (!is_first_lidar && cur_time < last_timestamp_lidar) {
    std::cerr << "lidar loop back, clear buffer" << std::endl;
    lidar_buffer.clear();
  }
  if (is_first_lidar) {
    is_first_lidar = false;
  }

  FastLioPointCloud::Ptr ptr(new FastLioPointCloud());
  p_pre->process(msg, ptr);
  lidar_buffer.push_back(ptr);
  time_buffer.push_back(cur_time);
  last_timestamp_lidar = cur_time;
}

void LidarProcess::process(const sensor_msgs::msg::PointCloud2::UniquePtr& msg,
                           FastLioPointCloud::Ptr& pcl_out) {
  FastLioPointCloudPtr fastlio_pc(new FastLioPointCloud());

  switch (lidar_type) {
    case LIVOX:
      livox_handler(msg, fastlio_pc);
      break;
    case VELODYNE:
      velodyne_handler(msg, fastlio_pc);
      break;
    case OUSTER:
      ouster_handler(msg, fastlio_pc);
      break;
    case HESAI:
      hesai_handler(msg, fastlio_pc);
      break;
  }

  std::vector<int> added_idxs, new_idxs;
  scan_min_extent = 0.02 * mean_range;

  ioctree.set_bucket_size(1);
  ioctree.set_min_extent(scan_min_extent);
  ioctree.initialize(pl_surf, added_idxs, new_idxs);
  *pcl_out = FastLioPointCloud(*fastlio_pc, added_idxs);
}

void LidarProcess::ouster_handler(
    const sensor_msgs::msg::PointCloud2::UniquePtr& msg,
    FastLioPointCloud::Ptr& fastlio_pc) {
  pcl::PointCloud<ouster_point> msg_pc;
  pcl::fromROSMsg(*msg, msg_pc);

  fastlio_pc->reserve(msg_pc.size());

  mean_range = 0.0;
  for (int i = 0; i < msg_pc.points.size(); i++) {
    double range = msg_pc.points[i].x * msg_pc.points[i].x +
                   msg_pc.points[i].y * msg_pc.points[i].y +
                   msg_pc.points[i].z * msg_pc.points[i].z;

    mean_range += sqrt(range);
  }
  mean_range /= msg_pc.points.size();

  double time_stamp = rclcpp::Time(msg->header.stamp).seconds();
  double dyn_range_min = fmin(min_range_, 0.1 * mean_range);
  double dyn_range_max = fmin(100, 10 * mean_range);

  for (int i = 0; i < msg_pc.points.size(); i++) {
    double range = msg_pc.points[i].x * msg_pc.points[i].x +
                   msg_pc.points[i].y * msg_pc.points[i].y +
                   msg_pc.points[i].z * msg_pc.points[i].z;

    if (sqrt(range) < dyn_range_min) continue;
    if (sqrt(range) > dyn_range_max) continue;

    FastLioPoint added_pt;
    added_pt.x = msg_pc.points[i].x;
    added_pt.y = msg_pc.points[i].y;
    added_pt.z = msg_pc.points[i].z;
    added_pt.intensity = msg_pc.points[i].intensity;
    added_pt.offset_time = msg_pc.points[i].t * time_unit_scale_;

    fastlio_pc->push_back(std::move(added_pt));
  }
}

void LidarProcess::velodyne_handler(
    const sensor_msgs::msg::PointCloud2::UniquePtr& msg,
    FastLioPointCloud::Ptr& fastlio_pc) {
  pcl::PointCloud<velodyne_point> msg_pc;
  pcl::fromROSMsg(*msg, msg_pc);

  fastlio_pc->reserve(msg_pc.size());

  mean_range = 0.0;
  for (int i = 0; i < msg_pc.points.size(); i++) {
    double range = msg_pc.points[i].x * msg_pc.points[i].x +
                   msg_pc.points[i].y * msg_pc.points[i].y +
                   msg_pc.points[i].z * msg_pc.points[i].z;

    mean_range += sqrt(range);
  }
  mean_range /= msg_pc.points.size();

  double time_stamp = rclcpp::Time(msg->header.stamp).seconds();
  double dyn_range_min = fmin(min_range_, 0.1 * mean_range);
  double dyn_range_max = fmin(100, 10 * mean_range);

  for (int i = 0; i < msg_pc.points.size(); i++) {
    double range = msg_pc.points[i].x * msg_pc.points[i].x +
                   msg_pc.points[i].y * msg_pc.points[i].y +
                   msg_pc.points[i].z * msg_pc.points[i].z;

    if (sqrt(range) < dyn_range_min) continue;
    if (sqrt(range) > dyn_range_max) continue;

    FastLioPoint added_pt;
    added_pt.x = msg_pc.points[i].x;
    added_pt.y = msg_pc.points[i].y;
    added_pt.z = msg_pc.points[i].z;
    added_pt.intensity = msg_pc.points[i].intensity;
    added_pt.offset_time = msg_pc.points[i].time * time_unit_scale_;

    fastlio_pc->push_back(std::move(added_pt));
  }
}

void LidarProcess::livox_handler(
    const sensor_msgs::msg::PointCloud2::UniquePtr& msg,
    FastLioPointCloud::Ptr& fastlio_pc) {
  pcl::PointCloud<livox_point> msg_pc;
  pcl::fromROSMsg(*msg, msg_pc);

  fastlio_pc->reserve(msg_pc.size());

  mean_range = 0.0;
  for (int i = 0; i < msg_pc.points.size(); i++) {
    double range = msg_pc.points[i].x * msg_pc.points[i].x +
                   msg_pc.points[i].y * msg_pc.points[i].y +
                   msg_pc.points[i].z * msg_pc.points[i].z;

    mean_range += sqrt(range);
  }
  mean_range /= msg_pc.points.size();

  double time_stamp = rclcpp::Time(msg->header.stamp).seconds();
  double dyn_range_min = fmin(min_range_, 0.1 * mean_range);
  double dyn_range_max = fmin(100, 10 * mean_range);

  for (int i = 0; i < msg_pc.points.size(); i++) {
    double range = msg_pc.points[i].x * msg_pc.points[i].x +
                   msg_pc.points[i].y * msg_pc.points[i].y +
                   msg_pc.points[i].z * msg_pc.points[i].z;

    if (sqrt(range) < dyn_range_min) continue;
    if (sqrt(range) > dyn_range_max) continue;

    FastLioPoint added_pt;
    added_pt.x = msg_pc.points[i].x;
    added_pt.y = msg_pc.points[i].y;
    added_pt.z = msg_pc.points[i].z;
    added_pt.intensity = msg_pc.points[i].reflectivity;
    added_pt.offset_time = msg_pc.points[i].offset_time * time_unit_scale_;

    fastlio_pc->push_back(std::move(added_pt));
  }
}

void LidarProcess::hesai_handler(
    const sensor_msgs::msg::PointCloud2::UniquePtr& msg,
    FastLioPointCloud::Ptr& fastlio_pc) {
  pcl::PointCloud<hesai_point> msg_pc;
  pcl::fromROSMsg(*msg, msg_pc);

  fastlio_pc->reserve(msg_pc.size());

  mean_range = 0.0;
  for (int i = 0; i < msg_pc.points.size(); i++) {
    double range = msg_pc.points[i].x * msg_pc.points[i].x +
                   msg_pc.points[i].y * msg_pc.points[i].y +
                   msg_pc.points[i].z * msg_pc.points[i].z;

    mean_range += sqrt(range);
  }
  mean_range /= msg_pc.points.size();

  double time_head = msg_pc.points[0].timestamp;
  double time_stamp = rclcpp::Time(msg->header.stamp).seconds();
  double dyn_range_min = fmin(min_range_, 0.1 * mean_range);
  double dyn_range_max = fmin(100, 10 * mean_range);

  for (int i = 0; i < msg_pc.points.size(); i++) {
    double range = msg_pc.points[i].x * msg_pc.points[i].x +
                   msg_pc.points[i].y * msg_pc.points[i].y +
                   msg_pc.points[i].z * msg_pc.points[i].z;

    if (sqrt(range) < dyn_range_min) continue;
    if (sqrt(range) > dyn_range_max) continue;

    FastLioPoint added_pt;
    added_pt.x = msg_pc.points[i].x;
    added_pt.y = msg_pc.points[i].y;
    added_pt.z = msg_pc.points[i].z;
    added_pt.intensity = msg_pc.points[i].intensity;
    added_pt.offset_time = msg_pc.points[i].timestamp - time_head;
    added_pt.offset_time *= time_unit_scale_;

    fastlio_pc->push_back(std::move(added_pt));
  }
}