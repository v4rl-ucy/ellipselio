#include "map_processing.h"

namespace ellipselio {

bool MappingNode::SyncPackages() {
  double velocity;
  bool got_lidar_data;
  KfState latest_state;

  auto& clk = *this->get_clock();
  double lidar_scan_time = 1.0 / lidar_params_.rate;
  double inter_sync_time = omp_get_wtime() - last_sync_time_;
  rclcpp::Duration lidar_scan_duration(0, 1e9 * lidar_scan_time);

  if (!last_sync_time_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), clk, 1000, "Waiting for data...");
  }
  if (!lid_process_->lidar_has_data_ && buffer_cloud_->empty() &&
      raw_cloud_->empty()) {
    if (inter_sync_time > 1.0) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), clk, 1000,
                            "Lidar has no new data");
    }
    return false;
  }
  imu_process_->lidar_ready_ = true;
  if (imu_process_->imu_end_time_ <= last_imu_time_) {
    if (inter_sync_time > 1.0) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), clk, 1000,
                            "IMU has no new data");
    }
    return false;
  }
  for (int i = 0; i < num_cams_; i++) {
    if (!cams_process_[i]->cam_has_data_) {
      if (inter_sync_time > 1.0) {
        RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), clk, 1000,
                                     "Camera " << i << " has no new data");
      }
    }
  }
  if (lid_process_->lidar_start_time_ < imu_process_->imu_start_time_) {
    lid_process_->ClearPointCloud();
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), clk, 1000,
                                 "Lidar start time is before IMU start time");
    return false;
  }

  got_lidar_data =
      lid_process_->GetPointCloud(raw_cloud_, &raw_start_time_, &raw_end_time_,
                                  &raw_cloud_bins_, &start_bin_, &mean_bin_);

  if (raw_cloud_->empty() && buffer_cloud_->empty()) {
    if (inter_sync_time > lidar_scan_time) {
      RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), clk, 1000,
                                   "No synced measurements");
    }
    return false;
  }

  imu_start_time_ = imu_process_->imu_start_time_;
  imu_end_time_ = imu_process_->imu_end_time_;
  last_imu_time_ = imu_process_->imu_end_time_;

  scan_start_time_ = buffer_start_time_;
  scan_end_time_ = raw_end_time_;

  if (buffer_cloud_->empty()) {
    scan_start_time_ = raw_start_time_;
  }
  if (raw_cloud_->empty()) {
    scan_end_time_ = buffer_end_time_;
  }
  if (imu_start_time_ > scan_start_time_) {
    scan_start_time_ = imu_start_time_;
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "IMU start time later than scan start time");
    buffer_cloud_->clear();
    return false;
  }
  if (scan_end_time_ - scan_start_time_ < lidar_scan_duration) {
    if (!buffer_cloud_->empty() || scan_end_time_ > imu_end_time_) {
      return false;
    }
  } else {
    if (scan_start_time_ + lidar_scan_duration > imu_end_time_) {
      return false;
    }
  }

  SyncRawCloudWithImu();

  int cur_imu_freq = round(imu_process_->imu_counter_ / inter_sync_time);
  int cur_lid_freq = round(lid_process_->lidar_counter_ / inter_sync_time);
  imu_process_->imu_counter_ = 0;
  lid_process_->lidar_counter_ = 0;

  analytics_msg_.imu_freq = cur_imu_freq;
  analytics_msg_.lid_freq = cur_lid_freq;

  analytics_msg_.cams_freq.clear();
  for (int i = 0; i < num_cams_; i++) {
    int cur_cam_freq = round(cams_process_[i]->cam_counter_ / inter_sync_time);
    cams_process_[i]->cam_counter_ = 0;
    analytics_msg_.cams_freq.push_back(cur_cam_freq);
  }

  int cur_odom_freq = round(1.0 / inter_sync_time);
  analytics_msg_.odom_freq = cur_odom_freq;

  last_sync_time_ = omp_get_wtime();
  return true;
}

void MappingNode::ComputeTensorVote(int i, int j, M3F* A_j, bool first_pass) {
  V3F p_i = map_cloud_->points[i].getVector3fMap();
  V3F p_j = map_cloud_->points[j].getVector3fMap();
  const int& bin_idx = map_cloud_->points[i].bin_idx;
  float search_rad = lid_process_->search_radii_[bin_idx];
  float d_ij = (p_i - p_j).norm();
  float c_ij = std::exp(-std::pow(d_ij, 2) / search_rad);
  V3F r_ij = (p_i - p_j).normalized();
  M3F rrt = r_ij * r_ij.transpose();
  M3F R_ij = Eye3f - 2.0 * rrt;
  M3F Rp_ij = (Eye3f - 0.5 * rrt) * R_ij;
  M3F K_j = Eye3f;
  if (!first_pass) K_j = tensors_p2_[j];
  *A_j = c_ij * R_ij * K_j * Rp_ij;
}

void MappingNode::ComputeTensorEigen(int i, M3F* tensor, bool first_pass) {
  V3F eig_val, sali_val;
  M3F eig_vec, tensor_i2;
  Eigen::SelfAdjointEigenSolver<M3F> eig_solver;

  eig_solver.computeDirect(*tensor);
  eig_vec = eig_solver.eigenvectors();
  eig_val = eig_solver.eigenvalues().cwiseAbs();

  const int& bin_idx = map_cloud_->points[i].bin_idx;
  const float& search_rad = lid_process_->search_radii_[bin_idx];

  if (first_pass) {
    tensor_i2 =
        (eig_val(2) - eig_val(1)) * eig_vec.col(2) * eig_vec.col(2).transpose();
    tensor_i2 += (eig_val(1) - eig_val(0)) *
                 (eig_vec.col(2) * eig_vec.col(2).transpose() +
                  eig_vec.col(1) * eig_vec.col(1).transpose());
    tensors_p2_[i] = tensor_i2;
  } else {
    sali_val(0) = eig_val(2) - eig_val(1);
    sali_val(1) = eig_val(1) - eig_val(0);
    sali_val(2) = eig_val(0);
    sali_val.maxCoeff(&saliency_idxs_[i]);

    filters_[i][1] = true;
    salivalues_[i] = sali_val;
    eigenvalues_[i] = (1.0 / (eig_val.array() + 1e-10)).matrix().normalized();
    eigenvalues_[i] *= search_rad;
    eigenvectors_[i] = eig_vec;
    map_cloud_->points[i].prim_type = (saliency_idxs_[i] + 1) * 85;
    if (num_cams_) return;
    switch (saliency_idxs_[i]) {
      case 0:
        map_cloud_->points[i].r = 32;
        map_cloud_->points[i].g = 144;
        map_cloud_->points[i].b = 240;
        break;
      case 1:
        map_cloud_->points[i].r = 94;
        map_cloud_->points[i].g = 201;
        map_cloud_->points[i].b = 98;
        break;
      case 2:
        map_cloud_->points[i].r = 253;
        map_cloud_->points[i].g = 231;
        map_cloud_->points[i].b = 36;
        break;
    }
  }
}

void MappingNode::TensorVotePass1(int old_map_size,
                                  std::vector<int>& added_idxs,
                                  std::vector<int>& updated_idxs) {
  int added_size = added_idxs.size();
  std::atomic<int> upd_idx = 0, new_neighbours_idx = 0;

#pragma omp parallel for
  for (int i = 0; i < added_size; i++) {
    int map_i;

    map_i = added_idxs[i];
    updated_pt_[map_i] = 0;
    map_cloud_->points[map_i].prim_type = 0;

    const int& bin_idx = map_cloud_->points[map_i].bin_idx;
    const int& bucket_size = lid_process_->bucket_sizes_[bin_idx];
    const float search_rad = lid_process_->search_radii_[bin_idx];

    neighbours_[map_i].reserve(lid_process_->max_neighbours_[bin_idx]);
    ioctree_.RadiusNeighbors(map_cloud_->points[map_i], search_rad,
                             neighbours_[map_i], bucket_size);

    n_bins_.row(i).setZero();
    n_cnts_.row(i).setZero();
    n_bins_(i, bin_idx) = 1;
    n_cnts_(i, bin_idx) = neighbours_[map_i].size();
  }

#pragma omp parallel for
  for (int i = 0; i < lid_process_->num_bins_; i++) {
    int n_bins_sum = n_bins_.col(i).head(added_size).sum();
    int tot_sum = lid_process_->cnt_neighbours_[i] + n_bins_sum;
    if (!n_bins_sum) continue;
    n_means_(i) = floor(n_means_(i) * lid_process_->cnt_neighbours_[i]);
    n_means_(i) += n_cnts_.col(i).head(added_size).sum();
    n_means_(i) = floor(n_means_(i) / tot_sum);
    n_means_(i) = fmin(fmax(n_means_(i), kMinNeighbours), kMaxNeighbours);
    lid_process_->min_neighbours_[i] = n_means_(i);
    lid_process_->max_neighbours_[i] = fmin(2 * n_means_(i), kMaxNeighbours);
    lid_process_->cnt_neighbours_[i] += n_bins_sum;
  }

#pragma omp parallel for
  for (int i = 0; i < added_idxs.size(); i++) {
    int map_i, loop_cnt;
    Eigen::MatrixXf K;
    M3F tensor_i1;

    map_i = added_idxs[i];

    const int& bin_idx = map_cloud_->points[map_i].bin_idx;
    const int& min_neigh = lid_process_->min_neighbours_[bin_idx];
    const int& max_neigh = lid_process_->max_neighbours_[bin_idx];

    loop_cnt = std::min(static_cast<int>(neighbours_[map_i].size()), max_neigh);
    K = Eigen::MatrixXf::Zero(loop_cnt, 9);

#pragma omp parallel for
    for (int j = 0; j < loop_cnt; j++) {
      M3F A_j;
      int map_j = neighbours_[map_i][j];
      ComputeTensorVote(map_i, map_j, &A_j, true);
      K.row(j) = A_j.reshaped(1, 9);

      if (map_j >= old_map_size) continue;

      const int& bin_idx_j = map_cloud_->points[map_j].bin_idx;
      float search_rad_j = lid_process_->search_radii_[bin_idx_j];
      EllipseLioPoint& pt_i = map_cloud_->points[map_i];
      EllipseLioPoint& pt_j = map_cloud_->points[map_j];
      float d_ij = (pt_i.getVector3fMap() - pt_j.getVector3fMap()).norm();

      if (d_ij > search_rad_j) continue;

      if (!(updated_pt_[map_j]++)) {
        update_idx_[map_j] = new_neighbours_idx++;
        new_neighbours_map_idx_[update_idx_[map_j]] = map_j;
        new_neighbours_size_[update_idx_[map_j]] = 0;
        new_neighbours_[update_idx_[map_j]]
                       [new_neighbours_size_[update_idx_[map_j]]++] = map_i;
      } else if (new_neighbours_size_[update_idx_[map_j]] < kMaxNeighbours) {
        new_neighbours_[update_idx_[map_j]]
                       [new_neighbours_size_[update_idx_[map_j]]++] = map_i;
      }
    }
    tensors_p1_[map_i] = K.colwise().sum().reshaped(3, 3);

    filters_[map_i][0] = loop_cnt >= min_neigh;
    if (!filters_[map_i][0]) continue;

    tensor_i1 = tensors_p1_[map_i] / float(loop_cnt);
    ComputeTensorEigen(map_i, &tensor_i1, true);
  }

  updated_idxs.resize(new_neighbours_idx);

#pragma omp parallel for
  for (int i = 0; i < new_neighbours_idx; i++) {
    Eigen::MatrixXf K;
    M3F tensor_i1;
    int map_i, loop_cnt, max_loop, old_size;

    map_i = new_neighbours_map_idx_[i];
    updated_pt_[map_i] = 0;

    const int& bin_idx = map_cloud_->points[map_i].bin_idx;
    const int& min_neigh = lid_process_->min_neighbours_[bin_idx];
    const int& max_neigh = lid_process_->max_neighbours_[bin_idx];
    if (neighbours_[map_i].size() >= max_neigh) continue;

    updated_idxs[upd_idx++] = map_i;

    max_loop = max_neigh - neighbours_[map_i].size();
    loop_cnt = std::min(static_cast<int>(new_neighbours_size_[i]), max_loop);

    old_size = neighbours_[map_i].size();
    neighbours_[map_i].resize(old_size + loop_cnt);

    K = Eigen::MatrixXf::Zero(loop_cnt, 9);

#pragma omp parallel for
    for (int j = 0; j < loop_cnt; j++) {
      M3F A_j;
      int map_j = new_neighbours_[i][j];
      neighbours_[map_i][old_size + j] = map_j;
      ComputeTensorVote(map_i, map_j, &A_j, true);
      K.row(j) = A_j.reshaped(1, 9);
    }

    tensors_p1_[map_i] += K.colwise().sum().reshaped(3, 3);

    filters_[map_i][0] = neighbours_[map_i].size() >= min_neigh;
    if (!filters_[map_i][0]) continue;

    tensor_i1 = tensors_p1_[map_i] / float(neighbours_[map_i].size());
    ComputeTensorEigen(map_i, &tensor_i1, true);
  }
  updated_idxs.resize(upd_idx);
}

void MappingNode::TensorVotePass2(std::vector<int>& added_idxs,
                                  std::vector<int>& updated_idxs) {
  int total_size = added_idxs.size() + updated_idxs.size();

#pragma omp parallel for
  for (int i = 0; i < total_size; i++) {
    M3F tensor_i2;
    Eigen::MatrixXf K;
    Eigen::VectorXi K_filter;
    int map_i, loop_cnt, filter_cnt;

    map_i = i < added_idxs.size() ? added_idxs[i]
                                  : updated_idxs[i - added_idxs.size()];

    const int& bin_idx = map_cloud_->points[map_i].bin_idx;
    const int& min_neigh = lid_process_->min_neighbours_[bin_idx];
    const int& max_neigh = lid_process_->max_neighbours_[bin_idx];
    loop_cnt = std::min(static_cast<int>(neighbours_[map_i].size()), max_neigh);

    if (!filters_[map_i][0]) continue;

    K = Eigen::MatrixXf::Zero(loop_cnt, 9);
    K_filter = Eigen::VectorXi::Zero(loop_cnt);

#pragma omp parallel for
    for (int j = 0; j < loop_cnt; j++) {
      int map_j = neighbours_[map_i][j];
      if (!filters_[map_j][0]) continue;

      M3F A_j;
      ComputeTensorVote(map_i, map_j, &A_j, false);
      K.row(j) = A_j.reshaped(1, 9);
      K_filter(j) = 1;
    }

    filter_cnt = K_filter.sum();
    if (filter_cnt < min_neigh) continue;

    tensor_i2 = K.colwise().sum().reshaped(3, 3);
    tensor_i2 /= float(filter_cnt);
    ComputeTensorEigen(map_i, &tensor_i2, false);
  }
}

void MappingNode::MapIncremental() {
  int start_idx, end_idx;
  std::vector<int> new_idxs, updated_idxs, added_idxs, map_idxs;

  poses_.push_back(kf_state_.state.pos.cast<float>());
  rotes_.push_back(kf_state_.state.rot.cast<float>());

  if (traj_dist_.empty()) {
    traj_dist_.push_back(0.0f);
  } else {
    float curr_dist = (poses_.back() - poses_[poses_.size() - 2]).norm();
    traj_dist_.push_back(traj_dist_.back() + curr_dist);
  }

  if (kf_state_.state.vel.norm() > 0.1 || vel_poses_.empty()) {
    vel_poses_.push_back(kf_state_.state.pos.cast<float>());
  }

#pragma omp parallel for
  for (int i = 0; i < scan_cloud_->size(); i++) {
    scan_cloud_->points[i].scan_idx = map_counter_;
    const int& bin_idx = scan_cloud_->points[i].bin_idx;
    scan_cloud_->points[i].bin_idx = fmax(bin_idx, start_bin_);
    scan_cloud_->points[i].getVector3fMap() =
        (kf_state_.state.rot *
             (kf_state_.state.offset_R_L_I *
                  scan_cloud_->points[i].getVector3fMap().cast<double>() +
              kf_state_.state.offset_T_L_I) +
         kf_state_.state.pos)
            .cast<float>();
  }

  start_idx = 0;
  end_idx = 0;
  old_map_size_ = map_cloud_->size();

  for (int i = 0; i < scan_cloud_bins_.size(); i++) {
    end_idx += scan_cloud_bins_[i];
    if (!scan_cloud_bins_[i]) continue;
    if (end_idx > scan_cloud_->size()) break;

    ioctree_.SetBucketSize(lid_process_->bucket_sizes_[fmax(i, start_bin_)]);
    ioctree_.Update(*scan_cloud_, added_idxs, map_idxs, start_idx, end_idx,
                    map_resolution_);
    *map_cloud_ += EllipseLioPointCloud(*scan_cloud_, added_idxs);
    new_idxs.insert(new_idxs.end(), map_idxs.begin(), map_idxs.end());
    start_idx = end_idx;
  }

  new_map_size_ = map_cloud_->size();

  update_idx_.resize(map_cloud_->size(), 0);
  saliency_idxs_.resize(map_cloud_->size(), 0);
  neighbours_.resize(map_cloud_->size(), std::vector<int>());
  filters_.resize(map_cloud_->size(), Eigen::Vector2i::Zero());

  tensors_p1_.resize(map_cloud_->size(), M3F::Zero());
  tensors_p2_.resize(map_cloud_->size(), M3F::Zero());
  salivalues_.resize(map_cloud_->size(), V3F::Zero());
  eigenvalues_.resize(map_cloud_->size(), V3F::Zero());
  eigenvectors_.resize(map_cloud_->size(), M3F::Zero());

  if (new_idxs.size() > 0) {
    TensorVotePass1(old_map_size_, new_idxs, updated_idxs);
    TensorVotePass2(new_idxs, updated_idxs);
  }

  analytics_msg_.map_size = map_cloud_->size();
  analytics_msg_.oct_num = ioctree_.OctantSize();
  analytics_msg_.new_idxs = new_idxs.size();
  analytics_msg_.upd_idxs = updated_idxs.size();
  analytics_msg_.traj_dist = traj_dist_.back();
}

void MappingNode::SplitMap(const sensor_msgs::msg::PointCloud2& input,
                           std::vector<sensor_msgs::msg::PointCloud2>& clouds,
                           size_t n) {
  const size_t total_points = input.width * input.height;
  const size_t point_step = input.point_step;
  const size_t chunk_size = (total_points + n - 1) / n;

  for (size_t i = 0; i < n && i * chunk_size < total_points; ++i) {
    size_t start_point = i * chunk_size;
    size_t end_point = std::min(start_point + chunk_size, total_points);
    size_t num_points = end_point - start_point;

    sensor_msgs::msg::PointCloud2 part;
    part.header = input.header;
    part.fields = input.fields;
    part.is_bigendian = input.is_bigendian;
    part.point_step = input.point_step;
    part.height = 1;
    part.width = static_cast<uint32_t>(num_points);
    part.is_dense = input.is_dense;
    part.row_step = part.point_step * part.width;
    part.data.resize(part.row_step);

    std::copy(input.data.begin() + start_point * point_step,
              input.data.begin() + end_point * point_step, part.data.begin());

    clouds.push_back(std::move(part));
  }
}

void MappingNode::PublishMap() {
  if (!map_counter_) return;

  sensor_msgs::msg::PointCloud2 map_msg;
  std::vector<sensor_msgs::msg::PointCloud2> map_parts;
  if (!map_cloud_->size()) return;

  map_mutex_.lock();
  PublishMarkers();
  pcl::toROSMsg(*map_cloud_, map_msg);
  map_mutex_.unlock();

  map_msg.header.stamp = kf_state_pub_.time;
  map_msg.header.frame_id = node_namespace_ + "/odom_ellipselio";

  SplitMap(map_msg, map_parts, (1000 * pub_map_n_secs_) / 100);
  for (auto& part : map_parts) {
    pub_map_->publish(part);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
}

void MappingNode::PublishScan() {
  if (!map_counter_) return;

  sensor_msgs::msg::PointCloud2 scan_msg;
  pcl::toROSMsg(*scan_cloud_pub_, scan_msg);
  scan_msg.header.stamp = kf_state_pub_.time;
  scan_msg.header.frame_id = node_namespace_ + "/odom_ellipselio";
  pub_scan_->publish(scan_msg);
}

void MappingNode::PublishMarkers() {
  if (!map_counter_) return;

  std::atomic<int> marker_idx = 0;
  visualization_msgs::msg::MarkerArray marker_array;

  if (!map_cloud_->size()) return;

  int count_idx = std::ceil(0.01 * (new_map_size_ - last_map_size_));

  marker_array.markers.resize(count_idx);
#pragma omp parallel for
  for (int i = last_map_size_; i < new_map_size_; i += 100) {
    Eigen::Quaternionf quat;
    visualization_msgs::msg::Marker marker;

    int map_idx = i;
    if (!filters_[map_idx][1]) continue;

    marker.id = map_idx;
    marker.frame_locked = true;
    marker.lifetime = rclcpp::Duration(0, 0);
    marker.header.frame_id = node_namespace_ + "/odom_ellipselio";
    marker.header.stamp = kf_state_pub_.time;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    quat = eigenvectors_[map_idx];

    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();

    marker.pose.position.x = map_cloud_->points[map_idx].x;
    marker.pose.position.y = map_cloud_->points[map_idx].y;
    marker.pose.position.z = map_cloud_->points[map_idx].z;

    switch (saliency_idxs_[map_idx]) {
      case 0:
        marker.ns = "plane";
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = 2 * eigenvalues_[map_idx](0);
        marker.scale.y = 2 * eigenvalues_[map_idx](1);
        marker.scale.z = 2 * eigenvalues_[map_idx](2);
        marker.color.r = 32.0 / 255.0;
        marker.color.g = 144.0 / 255.0;
        marker.color.b = 240.0 / 255.0;
        break;
      case 1:
        marker.ns = "line";
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = 2 * eigenvalues_[map_idx](0);
        marker.scale.y = 2 * eigenvalues_[map_idx](1);
        marker.scale.z = 2 * eigenvalues_[map_idx](2);
        marker.color.r = 94.0 / 255.0;
        marker.color.g = 201.0 / 255.0;
        marker.color.b = 98.0 / 255.0;
        break;
      case 2:
        marker.ns = "ball";
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = 2 * eigenvalues_[map_idx](0);
        marker.scale.y = 2 * eigenvalues_[map_idx](1);
        marker.scale.z = 2 * eigenvalues_[map_idx](2);
        marker.color.r = 253.0 / 255.0;
        marker.color.g = 231.0 / 255.0;
        marker.color.b = 36.0 / 255.0;
        break;
    }
    marker_array.markers[marker_idx++] = marker;
  }
  last_map_size_ = new_map_size_;
  marker_array.markers.resize(marker_idx);
  pub_mark_->publish(marker_array);
}

void MappingNode::PublishImuOdometry() {
  KfState imu_state;
  nav_msgs::msg::Odometry odom_msg;

  if (!map_counter_) return;

  imu_process_->GetKfState(&imu_state);
  if (last_imu_pub_time == imu_state.time) return;
  last_imu_pub_time = imu_state.time;

  geometry_msgs::msg::TransformStamped trans;
  trans.header.frame_id = node_namespace_ + "/odom_ellipselio";
  trans.child_frame_id = node_namespace_ + "/imu_prop_ellipselio";
  trans.header.stamp = imu_state.time;
  trans.transform.translation.x = imu_state.state.pos(0);
  trans.transform.translation.y = imu_state.state.pos(1);
  trans.transform.translation.z = imu_state.state.pos(2);
  trans.transform.rotation.x = imu_state.state.rot.coeffs()[0];
  trans.transform.rotation.y = imu_state.state.rot.coeffs()[1];
  trans.transform.rotation.z = imu_state.state.rot.coeffs()[2];
  trans.transform.rotation.w = imu_state.state.rot.coeffs()[3];
  tf_br_->sendTransform(trans);

  odom_msg.header.frame_id = node_namespace_ + "/odom_ellipselio";
  odom_msg.child_frame_id = node_namespace_ + "/imu_prop_ellipselio";
  odom_msg.header.stamp = imu_state.time;

  odom_msg.pose.pose.position.x = imu_state.state.pos(0);
  odom_msg.pose.pose.position.y = imu_state.state.pos(1);
  odom_msg.pose.pose.position.z = imu_state.state.pos(2);
  odom_msg.pose.pose.orientation.x = imu_state.state.rot.coeffs()[0];
  odom_msg.pose.pose.orientation.y = imu_state.state.rot.coeffs()[1];
  odom_msg.pose.pose.orientation.z = imu_state.state.rot.coeffs()[2];
  odom_msg.pose.pose.orientation.w = imu_state.state.rot.coeffs()[3];

  V3D lin_vel_body = imu_state.state.rot.conjugate() * imu_state.state.vel;
  V3D ang_vel_body = imu_state.gyr;

  odom_msg.twist.twist.linear.x = lin_vel_body(0);
  odom_msg.twist.twist.linear.y = lin_vel_body(1);
  odom_msg.twist.twist.linear.z = lin_vel_body(2);
  odom_msg.twist.twist.angular.x = ang_vel_body(0);
  odom_msg.twist.twist.angular.y = ang_vel_body(1);
  odom_msg.twist.twist.angular.z = ang_vel_body(2);

  Eigen::MatrixXd pose_cov = imu_state.cov.block<6, 6>(0, 0);
  Eigen::VectorXd pose_cov_vec = pose_cov.reshaped<Eigen::RowMajor>();
  Eigen::MatrixXd twist_cov = Eigen::MatrixXd::Zero(6, 6);
  twist_cov.block<3, 3>(0, 0) = imu_state.cov.block<3, 3>(12, 12);
  twist_cov.block<3, 3>(3, 3) = imu_process_->q_.block<3, 3>(0, 0);
  Eigen::VectorXd twist_cov_vec = twist_cov.reshaped<Eigen::RowMajor>();

#pragma omp parallel for
  for (int i = 0; i < 36; i++) {
    odom_msg.pose.covariance[i] = pose_cov_vec(i);
    odom_msg.twist.covariance[i] = twist_cov_vec(i);
  }
  pub_odom_->publish(odom_msg);
}

void MappingNode::PublishLidarOdometry() {
  if (!map_counter_) return;

  if (last_opt_pub_time == kf_state_pub_.time) return;
  odom_mutex_.lock();
  last_opt_pub_time = kf_state_pub_.time;

  geometry_msgs::msg::TransformStamped trans;
  trans.header.frame_id = node_namespace_ + "/odom_ellipselio";
  trans.child_frame_id = node_namespace_ + "/imu_ellipselio";
  trans.header.stamp = kf_state_pub_.time;
  trans.transform.translation.x = kf_state_pub_.state.pos(0);
  trans.transform.translation.y = kf_state_pub_.state.pos(1);
  trans.transform.translation.z = kf_state_pub_.state.pos(2);
  trans.transform.rotation.x = kf_state_pub_.state.rot.coeffs()[0];
  trans.transform.rotation.y = kf_state_pub_.state.rot.coeffs()[1];
  trans.transform.rotation.z = kf_state_pub_.state.rot.coeffs()[2];
  trans.transform.rotation.w = kf_state_pub_.state.rot.coeffs()[3];
  tf_br_->sendTransform(trans);

  pub_analytics_->publish(analytics_msg_pub_);
  PublishScan();
  odom_mutex_.unlock();
}

void MappingNode::TensorRegistration(
    state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) {
  double t0, t1, res_mean;
  int feat_tot, reject_cnt, start_bin_scale;
  float grav_check, vert_score;
  float wt_min, wt_max, wt_mean, wt_std, obs_min, obs_scale, obs_term;
  float rng_min, rng_max, rng_mean, rng_scale, rng_min_scale, rng_max_scale;

  Eigen::Array3i cnts(3);
  std::atomic<int> feat_cnt;
  std::vector<std::atomic<int>> prim_cnts(3);

  M3F cov_mat;
  V3D rot_obs, tran_obs, cov_scales;
  V3F grav_norm, poses_diff, poses_orth, grav_x, grav_y, grav_z;

  Eigen::Quaternionf gq;
  Eigen::Matrix4f tf_grav;
  Eigen::Vector4f centroid = Eigen::Vector4f::Zero();

  feat_cnt = 0;
  prim_cnts[0] = 0;
  prim_cnts[1] = 0;
  prim_cnts[2] = 0;

  grav_norm = kf_state_.state.grav.get_vect().normalized().cast<float>();
  poses_diff = s.pos.cast<float>();
  poses_diff -= vel_poses_[fmax(vel_poses_.size() - 100, 0)];
  grav_check = fabs(grav_norm.dot(poses_diff));
  grav_check *= fabs(grav_norm.dot(poses_diff.normalized()));

  poses_orth = poses_diff - grav_norm * grav_norm.dot(poses_diff);
  vert_score = 1.0 - (((100.0 / mean_bin_) * grav_check) / poses_orth.norm());

  scan_cloud_grav_->resize(scan_cloud_->size());
  gq = Eigen::Quaternionf::FromTwoVectors(grav_norm, V3F::UnitZ());
  gq = gq * s.rot.cast<float>() * s.offset_R_L_I.cast<float>();

  tf_grav = Eigen::Matrix4f::Identity();
  tf_grav.block<3, 3>(0, 0) = gq.toRotationMatrix();

  gq = Eigen::Quaternionf::FromTwoVectors(V3F::UnitZ(), grav_norm);
  grav_x = gq.toRotationMatrix().col(0);
  grav_y = gq.toRotationMatrix().col(1);
  grav_z = gq.toRotationMatrix().col(2);

  pcl::transformPointCloud(*scan_cloud_, *scan_cloud_grav_, tf_grav);
  pcl::compute3DCentroid(*scan_cloud_grav_, centroid);
  pcl::computeCovarianceMatrixNormalized(*scan_cloud_grav_, centroid, cov_mat);

  float mean_bin_arc = lidar_params_.vertical_fov * kPi * mean_bin_ / 180.0;
  float cov_score = fmin(fabs(cov_mat(2, 2)) / mean_bin_arc, 1.0);

  cov_scales = cov_mat.diagonal().cast<double>();
  cov_scales(2) *= cov_score;
  cov_scales(2) *= 180.0 / lidar_params_.vertical_fov;
  cov_scales /= cov_scales.maxCoeff();

  start_bin_scale = floor(kMaxSearchRes / lid_process_->match_radii_.front());

  t0 = omp_get_wtime();

#pragma omp parallel for
  for (int i = 0; i < scan_cloud_->size(); i++) {
    Eigen::VectorXd h_x_vec(6);
    Eigen::VectorXd::Index tran_idx, rot_idx;
    std::vector<int> N_idxs;
    std::vector<float> N_dst;
    rclcpp::Time map_pt_time, scan_pt_time;
    int sali_idx, map_i, feat_num, prim_num;
    float inc_search_rad, octree_res, residual, time_score, traj_diff,
        curr_traj_dist, bin_scale;

    M3D P_skew;
    V3D p_lidar, p_imu, a, obs_trans, obs_rot, obs_idx_trans, obs_idx_rot;
    V3F scores, p_world, n_world, p_dash, q, q_dash, norm_vec, a_world;

    const EllipseLioPoint& pt = scan_cloud_->points[i];

    p_lidar = pt.getVector3fMap().cast<double>();
    p_imu = s.offset_R_L_I * p_lidar + s.offset_T_L_I;
    p_world = (s.rot * p_imu + s.pos).cast<float>();

    const int& bin_idx = scan_cloud_->points[i].bin_idx;
    const float& match_rad = lid_process_->match_radii_[bin_idx];
    const float& search_rad = lid_process_->search_radii_[bin_idx];
    const float& min_oct_res = lid_process_->octree_resolutions_.front();

    inc_search_rad = match_rad / (ekfom_iter_cnt_ + 1);
    inc_search_rad = fmax(fmin(inc_search_rad, kMaxSearchRes), min_oct_res);

    curr_traj_dist = traj_dist_.back();
    curr_traj_dist += (s.pos.cast<float>() - poses_.back()).norm();
    if (inc_search_rad > curr_traj_dist) {
      inc_search_rad = fmax(0.5 * inc_search_rad, kMinSearchRes);
    }

    ioctree_.KnnNeighbors(p_world, 1, N_idxs, N_dst, inc_search_rad);
    if (N_idxs.empty()) continue;

    map_i = N_idxs[0];
    if (!filters_[map_i][1]) continue;

    traj_diff = curr_traj_dist;
    traj_diff -= traj_dist_[map_cloud_->points[map_i].scan_idx];
    bin_scale = fmax(fmin(bin_idx / 4.0, 10.0), start_bin_scale);
    if (map_cloud_->points[map_i].scan_idx && s.vel.norm() > 0.1 &&
        match_rad > bin_scale * search_rad &&
        traj_diff < match_rad / bin_scale) {
      continue;
    }

    scores = salivalues_[map_i] / salivalues_[map_i].sum();
    n_world = map_cloud_->points[map_i].getVector3fMap();
    q = p_world - n_world;

    q_dash = q.dot(eigenvectors_[map_i].col(2)) * eigenvectors_[map_i].col(2);
    p_dash = scores(0) * (p_world - q_dash);

    q_dash = q.dot(eigenvectors_[map_i].col(0)) * eigenvectors_[map_i].col(0);
    p_dash += scores(1) * (n_world + q_dash);

    p_dash += scores(2) * n_world;

    norm_vec = p_world - p_dash;
    residual = norm_vec.norm();
    norm_vec.normalize();

    time_score = 1.0 / (traj_diff + 1.0);
    time_score *= fmax(1.0 - fabs(grav_norm.dot(norm_vec)), 1e-4);
    time_score = 1.0 / fmin(fmax(time_score, 1e-4), 1.0);

    P_skew = SkewSymMat(p_imu);
    a = P_skew * s.rot.conjugate() * norm_vec.cast<double>();
    a_world = (s.rot * a).normalized().cast<float>();
    h_x_vec << norm_vec(0), norm_vec(1), norm_vec(2), a[0], a[1], a[2];

    obs_trans = cov_scales * pow(scores(0), 2);
    obs_trans(0) *= fabs(norm_vec.dot(grav_x));
    obs_trans(1) *= fabs(norm_vec.dot(grav_y));
    obs_trans(2) *= fabs(norm_vec.dot(grav_z));
    obs_rot(0) = fabs(a_world.dot(grav_x));
    obs_rot(1) = fabs(a_world.dot(grav_y));
    obs_rot(2) = fabs(a_world.dot(grav_z));

    obs_trans.maxCoeff(&tran_idx);
    obs_idx_trans = V3D::Zero();
    obs_idx_trans(tran_idx) = 1;
    obs_trans = obs_trans.cwiseProduct(obs_idx_trans);

    obs_rot.maxCoeff(&rot_idx);
    obs_idx_rot = V3D::Zero();
    obs_idx_rot(rot_idx) = 1;
    obs_rot = obs_rot.cwiseProduct(obs_idx_rot);

    feat_num = ++feat_cnt;
    sali_idx = saliency_idxs_[map_i];
    prim_num = ++prim_cnts[sali_idx];

    ekfom_data_ot_.row(feat_num - 1) = obs_trans;
    ekfom_data_or_.row(feat_num - 1) = obs_rot;
    ekfom_data_w_.row(feat_num - 1) = time_score;

    ekfom_data_h_v_(feat_num - 1) = -residual;
    ekfom_data_h_x_v_.row(feat_num - 1) = h_x_vec;
  }

  cnts << prim_cnts[0].load(), prim_cnts[1].load(), prim_cnts[2].load();
  feat_tot = feat_cnt.load();
  reject_cnt = scan_cloud_->size() - feat_tot;

  wt_min = ekfom_data_w_.head(feat_tot).minCoeff();
  wt_max = ekfom_data_w_.head(feat_tot).maxCoeff();
  wt_mean = ekfom_data_w_.head(feat_tot).mean();
  wt_std = (ekfom_data_w_.head(feat_tot) - wt_mean).square().sum();
  wt_std = sqrt(wt_std / (feat_tot - 1));

  rng_min = wt_mean - wt_std;
  rng_min = fmax(rng_min, wt_min);
  rng_min_scale = rng_min / wt_min;
  rng_mean = wt_mean - rng_min;
  rng_max = wt_max - rng_min;
  rng_max = fmin(rng_mean + wt_std, rng_max);
  rng_max_scale = rng_max / ((wt_max * rng_min_scale) - rng_min);

  if (wt_std) {
    ekfom_data_w_.head(feat_tot) *= rng_min_scale;
    ekfom_data_w_.head(feat_tot) -= rng_min;
    ekfom_data_w_.head(feat_tot) *= rng_max_scale;
    ekfom_data_w_.head(feat_tot) += 1.0;
  }

  tran_obs = ekfom_data_ot_.topRows(feat_tot).colwise().sum() + 1e-4;
  rot_obs = ekfom_data_or_.topRows(feat_tot).colwise().sum() + 1e-4;

  tran_obs /= tran_obs.maxCoeff();
  rot_obs /= rot_obs.maxCoeff();

  obs_min = 1e4 * rot_obs.minCoeff() * tran_obs.minCoeff();

  if (!ekfom_data_om_.sum()) {
    ekfom_data_om_ += fmin(fmax(obs_min, 1e-4), 1.0);
    ekfom_data_oe_ += fmin(fmax(vert_score, 1e-4), 1.0);
  } else {
    ekfom_data_om_[ekfom_obs_cnt_] = fmin(fmax(obs_min, 1e-4), 1.0);
    ekfom_data_oe_[ekfom_vert_cnt_] = fmin(fmax(vert_score, 1e-4), 1.0);
  }
  ekfom_obs_cnt_ = (ekfom_obs_cnt_ + 1) % ekfom_data_om_.size();
  ekfom_vert_cnt_ = (ekfom_vert_cnt_ + 1) % ekfom_data_oe_.size();

  obs_min =
      ekfom_data_om_.mean() * ekfom_data_oe_.mean() * ekfom_data_sb_.mean();
  obs_min = fmax(obs_min, 0.1);
  ekfom_data_w_.head(feat_tot) = ekfom_data_w_.head(feat_tot).pow(obs_min);
  ekfom_data_h_.block(0, 0, feat_tot, 1) = ekfom_data_h_v_.head(feat_tot);
  ekfom_data_w_x_.block(0, 0, feat_tot, 1) = ekfom_data_w_.head(feat_tot);
  ekfom_data_h_x_.block(0, 0, feat_tot, 6) =
      ekfom_data_h_x_v_.topRows(feat_tot);

  wt_min = ekfom_data_w_.head(feat_tot).minCoeff();
  wt_max = ekfom_data_w_.head(feat_tot).maxCoeff();
  wt_mean = ekfom_data_w_.head(feat_tot).mean();
  wt_std = (ekfom_data_w_.head(feat_tot) - wt_mean).square().sum();
  wt_std = sqrt(wt_std / (feat_tot - 1));

  rng_min = 1;
  rng_max += 1;
  rng_mean += 1;

  ekfom_data_h_x_r_.leftCols(feat_tot) =
      (ekfom_data_h_x_.topRows(feat_tot).array().colwise() *
       ekfom_data_w_x_.head(feat_tot))
          .transpose();

  res_mean = -ekfom_data_h_.head(feat_tot).sum() / feat_tot;
  ekfom_data.h = ekfom_data_h_.head(feat_tot);
  ekfom_data.h_x = ekfom_data_h_x_.topRows(feat_tot);
  ekfom_data.h_x_R = ekfom_data_h_x_r_.leftCols(feat_tot);

  ekfom_iter_cnt_++;

  analytics_msg_.num_planes = cnts(0);
  analytics_msg_.num_lines = cnts(1);
  analytics_msg_.num_balls = cnts(2);
  analytics_msg_.wt_std = wt_std;
  analytics_msg_.wt_min = wt_min;
  analytics_msg_.wt_max = wt_max;
  analytics_msg_.obs_min = obs_min;
  analytics_msg_.wt_mean = wt_mean;
  analytics_msg_.rng_min = rng_min;
  analytics_msg_.rng_max = rng_max;
  analytics_msg_.rng_mean = rng_mean;
  analytics_msg_.res_mean = res_mean;
  analytics_msg_.mean_bin = mean_bin_;
  analytics_msg_.start_bin = start_bin_;
  analytics_msg_.num_feats = feat_tot;
  analytics_msg_.num_reject = reject_cnt;
  analytics_msg_.kf_iterations = ekfom_iter_cnt_;
  analytics_msg_.obs_score = ekfom_data_om_.mean();
  analytics_msg_.vert_score = ekfom_data_oe_.mean();
  analytics_msg_.bin_score = ekfom_data_sb_.mean();
}

MappingNode::MappingNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("mapping_node", options),
      map_cloud_(new EllipseLioPointCloud()),
      raw_cloud_(new EllipseLioPointCloud()),
      scan_cloud_(new EllipseLioPointCloud()),
      scan_cloud_grav_(new EllipseLioPointCloud()),
      filter_cloud_(new EllipseLioPointCloud()),
      buffer_cloud_(new EllipseLioPointCloud()),
      scan_cloud_pub_(new EllipseLioPointCloud()),
      kf_(new Ikfom()) {
  this->declare_parameter<std::string>("mapping.namespace", "");
  this->declare_parameter<int>("mapping.pub_map_n_secs", 10);
  this->declare_parameter<double>("mapping.map_resolution", 0.1);

  this->declare_parameter<int>("imu.rate", 100);
  this->declare_parameter<double>("imu.gyr_noise", 0.1);
  this->declare_parameter<double>("imu.acc_noise", 0.1);
  this->declare_parameter<double>("imu.gyr_bias", 0.0001);
  this->declare_parameter<double>("imu.acc_bias", 0.0001);
  this->declare_parameter<std::string>("imu.topic", "");

  this->declare_parameter<int>("lidar.type", 0);
  this->declare_parameter<int>("lidar.rate", 10);
  this->declare_parameter<int>("lidar.scan_lines", 64);
  this->declare_parameter<double>("lidar.min_range", 1.0);
  this->declare_parameter<double>("lidar.max_range", 100.0);
  this->declare_parameter<double>("lidar.vertical_fov", 64.0);
  this->declare_parameter<std::string>("lidar.topic", "");

  this->declare_parameter<std::vector<double>>("lidar.t_imu_lidar",
                                               std::vector<double>());
  this->declare_parameter<std::vector<double>>("lidar.r_imu_lidar",
                                               std::vector<double>());

  this->declare_parameter<int>("cameras.num_cams", 0);
  this->declare_parameter<std::string>("cameras.transport", "raw");
  this->declare_parameter<std::vector<long int>>("cameras.frame_rates",
                                                 std::vector<long int>());
  this->declare_parameter<std::vector<std::string>>("cameras.cam_topics",
                                                    std::vector<std::string>());
  this->declare_parameter<std::vector<double>>("cameras.cam_intrinsics",
                                               std::vector<double>());
  this->declare_parameter<std::vector<double>>("cameras.t_cam_lidars",
                                               std::vector<double>());
  this->declare_parameter<std::vector<double>>("cameras.r_cam_lidars",
                                               std::vector<double>());

  this->get_parameter_or<std::string>("mapping.namespace", node_namespace_, "");
  this->get_parameter_or<int>("mapping.pub_map_n_secs", pub_map_n_secs_, 10);
  this->get_parameter_or<double>("mapping.map_resolution", map_resolution_,
                                 0.1);

  this->get_parameter_or<int>("imu.rate", imu_params_.rate, 100);
  this->get_parameter_or<double>("imu.gyr_noise", imu_params_.gyr_noise, 0.1);
  this->get_parameter_or<double>("imu.acc_noise", imu_params_.acc_noise, 0.1);
  this->get_parameter_or<double>("imu.gyr_bias", imu_params_.gyr_bias, 0.0001);
  this->get_parameter_or<double>("imu.acc_bias", imu_params_.acc_bias, 0.0001);
  this->get_parameter_or<std::string>("imu.topic", imu_params_.topic, "");

  this->get_parameter_or<int>("lidar.type", lidar_params_.type, 0);
  this->get_parameter_or<int>("lidar.rate", lidar_params_.rate, 10);
  this->get_parameter_or<int>("lidar.scan_lines", lidar_params_.scan_lines, 64);
  this->get_parameter_or<std::string>("lidar.topic", lidar_params_.topic, "");
  this->get_parameter_or<double>("lidar.min_range", lidar_params_.min_range,
                                 1.0);
  this->get_parameter_or<double>("lidar.max_range", lidar_params_.max_range,
                                 100.0);
  this->get_parameter_or<double>("lidar.vertical_fov",
                                 lidar_params_.vertical_fov, 64.0);

  this->get_parameter_or<std::vector<double>>("lidar.t_imu_lidar", t_imu_lidar_,
                                              std::vector<double>());
  this->get_parameter_or<std::vector<double>>("lidar.r_imu_lidar", r_imu_lidar_,
                                              std::vector<double>());

  this->get_parameter_or<int>("cameras.num_cams", num_cams_, 0);
  this->get_parameter_or<std::string>("cameras.transport", cam_transport_,
                                      "raw");
  this->get_parameter_or<std::vector<long int>>(
      "cameras.frame_rates", cam_frame_rates_, std::vector<long int>());
  this->get_parameter_or<std::vector<std::string>>(
      "cameras.cam_topics", cam_topics_, std::vector<std::string>());
  this->get_parameter_or<std::vector<double>>(
      "cameras.cam_intrinsics", cam_intrinsics_, std::vector<double>());
  this->get_parameter_or<std::vector<double>>(
      "cameras.t_cam_lidars", t_cam_lidars_, std::vector<double>());
  this->get_parameter_or<std::vector<double>>(
      "cameras.r_cam_lidars", r_cam_lidars_, std::vector<double>());

  map_cloud_->reserve(kMaxMapPoints);
  raw_cloud_->reserve(kMaxProcPoints);
  scan_cloud_->reserve(kMaxProcPoints);
  scan_cloud_grav_->reserve(kMaxProcPoints);
  filter_cloud_->reserve(kMaxProcPoints);
  buffer_cloud_->reserve(kMaxProcPoints);

  map_resolution_ = fmax(map_resolution_, kMinMapRes);
  map_search_rad_ = 10 * map_resolution_;

  ioctree_.SetBucketSize(1);
  ioctree_.SetMaxOctants(kMaxMapPoints);
  ioctree_.SetMaxNewPoints(kMaxProcPoints);
  ioctree_.SetMinExtent(map_resolution_);

  ekfom_data_sb_ = Eigen::ArrayXd::Zero(100);
  ekfom_data_om_ = Eigen::ArrayXd::Zero(100);
  ekfom_data_oe_ = Eigen::ArrayXd::Zero(100);
  ekfom_data_w_ = Eigen::ArrayXd(kMaxProcPoints);
  ekfom_data_ot_ = Eigen::ArrayXXd(kMaxProcPoints, 3);
  ekfom_data_or_ = Eigen::ArrayXXd(kMaxProcPoints, 3);
  ekfom_data_h_ = Eigen::VectorXd(kMaxProcPoints);
  ekfom_data_w_x_ = Eigen::ArrayXd(kMaxProcPoints);
  ekfom_data_h_x_ = Eigen::MatrixXd(kMaxProcPoints, 6);
  ekfom_data_h_x_r_ = Eigen::MatrixXd(6, kMaxProcPoints);
  ekfom_data_h_v_ = Eigen::ArrayXd(kMaxProcPoints);
  ekfom_data_h_x_v_ = Eigen::MatrixXd(kMaxProcPoints, 6);

  update_idx_.reserve(kMaxMapPoints);
  saliency_idxs_.reserve(kMaxMapPoints);
  neighbours_.reserve(kMaxMapPoints);
  filters_.reserve(kMaxMapPoints);

  tensors_p1_.reserve(kMaxMapPoints);
  tensors_p2_.reserve(kMaxMapPoints);
  salivalues_.reserve(kMaxMapPoints);
  eigenvalues_.reserve(kMaxMapPoints);
  eigenvectors_.reserve(kMaxMapPoints);

  updated_pt_ = std::vector<std::atomic<int>>(kMaxMapPoints);
  new_neighbours_map_idx_ = std::vector<int>(kMaxScanPoints);
  new_neighbours_size_ = std::vector<std::atomic<int>>(kMaxScanPoints);
  new_neighbours_ = std::vector<std::vector<int>>(
      kMaxScanPoints, std::vector<int>(kMaxNeighbours));

  analytics_msg_ = ellipselio::msg::EllipseLioAnalytics();
  analytics_msg_pub_ = ellipselio::msg::EllipseLioAnalytics();

  last_imu_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  imu_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  imu_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  raw_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  raw_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  scan_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  scan_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  buffer_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  buffer_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  imu_params_.t_imu_lidar = Vec3dFromArray(t_imu_lidar_);
  if (r_imu_lidar_.size() == 9) {
    imu_params_.r_imu_lidar = Mat3dFromArray(r_imu_lidar_);
  } else if (r_imu_lidar_.size() == 4) {
    imu_params_.r_imu_lidar =
        QuaterniondFromArray(r_imu_lidar_).toRotationMatrix();
  } else {
    RCLCPP_ERROR(
        this->get_logger(),
        "Lidar to IMU rotation is not a valid quaternion or rotation matrix");
  }

  kf_->init_dyn_share(GetF, DfDx, DfDw,
                      std::bind(&MappingNode::TensorRegistration, this,
                                std::placeholders::_1, std::placeholders::_2),
                      10, 1e-3);

  last_imu_pub_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_opt_pub_time = rclcpp::Time(0, 0, RCL_ROS_TIME);

  loop_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pub_map_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pub_odom_lid_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pub_odom_imu_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      node_namespace_ + "/ellipselio_odom", rclcpp::SensorDataQoS());
  pub_analytics_ = this->create_publisher<ellipselio::msg::EllipseLioAnalytics>(
      node_namespace_ + "/analytics", rclcpp::SensorDataQoS());
  pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      node_namespace_ + "/cloud_map", rclcpp::SensorDataQoS());
  pub_scan_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      node_namespace_ + "/cloud_scan", rclcpp::SensorDataQoS());
  pub_mark_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      node_namespace_ + "/visualization_marker", rclcpp::SensorDataQoS());

  loop_timer_ = rclcpp::create_timer(
      this, this->get_clock(),
      std::chrono::milliseconds(std::max(1000 / imu_params_.rate, 10)),
      std::bind(&MappingNode::TimerCallback, this), loop_callback_group_);
  pub_odom_lid_timer_ = rclcpp::create_timer(
      this, this->get_clock(),
      std::chrono::milliseconds(std::max(1000 / lidar_params_.rate, 10)),
      std::bind(&MappingNode::PublishLidarOdometry, this),
      pub_odom_lid_callback_group_);
  pub_odom_imu_timer_ = rclcpp::create_timer(
      this, this->get_clock(),
      std::chrono::milliseconds(std::max(1000 / imu_params_.rate, 10)),
      std::bind(&MappingNode::PublishImuOdometry, this),
      pub_odom_imu_callback_group_);
  pub_map_timer_ = rclcpp::create_timer(
      this, this->get_clock(),
      std::chrono::milliseconds(std::max(pub_map_n_secs_ * 1000, 10)),
      std::bind(&MappingNode::PublishMap, this), pub_map_callback_group_);

  char line[128];
  struct tms timeSample;
  last_cpu_ = times(&timeSample);
  last_sys_cpu_ = timeSample.tms_stime;
  last_user_cpu_ = timeSample.tms_utime;

  FILE* file;
  file = fopen("/proc/cpuinfo", "r");
  num_processors_ = 0;
  while (fgets(line, 128, file) != nullptr) {
    if (strncmp(line, "processor", 9) == 0) num_processors_++;
  }
  fclose(file);

  start_time_ = omp_get_wtime();
  RCLCPP_INFO(this->get_logger(), "Node init finished.");
}

MappingNode::~MappingNode() {}

void MappingNode::InitCamProcess() {
  if (num_cams_ == 0) return;

  if (cam_frame_rates_.size() != num_cams_) {
    RCLCPP_ERROR(this->get_logger(), "Frame rates and num cameras mismatch");
    exit(1);
  }
  if (cam_topics_.size() != num_cams_) {
    RCLCPP_ERROR(this->get_logger(), "Cam topics and num cameras mismatch");
    exit(1);
  }
  if (t_cam_lidars_.size() != num_cams_ * 3) {
    RCLCPP_ERROR(this->get_logger(),
                 "Cam translations and num cameras mismatch");
    exit(1);
  }
  if (!(r_cam_lidars_.size() == num_cams_ * 9 ||
        r_cam_lidars_.size() == num_cams_ * 4)) {
    RCLCPP_ERROR(this->get_logger(), "Cam rotations and num cameras mismatch");
    exit(1);
  }
  if (cam_intrinsics_.size() != num_cams_ * 9) {
    RCLCPP_ERROR(this->get_logger(), "Cam intrinsics and num cameras mismatch");
    exit(1);
  }

  for (int i = 0; i < num_cams_; i++) {
    CamParams cam_params;

    cam_params.topic = cam_topics_[i];
    cam_params.rate = cam_frame_rates_[i];
    cam_params.transport = cam_transport_;

    std::vector<double> t_cam_lidar(t_cam_lidars_.begin() + i * 3,
                                    t_cam_lidars_.begin() + i * 3 + 3);
    std::vector<double> cam_intrinsic(cam_intrinsics_.begin() + i * 9,
                                      cam_intrinsics_.begin() + i * 9 + 9);

    cam_params.t_cam_lidar = Vec3dFromArray(t_cam_lidar);
    cam_params.cam_intrinsics = Mat3dFromArray(cam_intrinsic);

    if (r_cam_lidars_.size() == num_cams_ * 4) {
      std::vector<double> r_cam_lidar(r_cam_lidars_.begin() + i * 4,
                                      r_cam_lidars_.begin() + i * 4 + 4);
      cam_params.r_cam_lidar =
          QuaterniondFromArray(r_cam_lidar).toRotationMatrix();
    } else {
      std::vector<double> r_cam_lidar(r_cam_lidars_.begin() + i * 9,
                                      r_cam_lidars_.begin() + i * 9 + 9);
      cam_params.r_cam_lidar = Mat3dFromArray(r_cam_lidar);
    }

    cams_process_.push_back(
        std::make_shared<CamProcess>(cam_params, shared_from_this()));
  }
}

void MappingNode::SyncRawCloudWithImu() {
  int total_size;
  std::atomic<int> scan_idx = 0, filter_idx = 0;

  imu_time_offset_ = 0;
  imu_process_->SyncWithLidar(&imu_start_time_, &imu_end_time_);
  lid_time_offset_ = lid_process_->lidar_time_offset_.seconds();

  if (!raw_cloud_->empty()) {
    scan_start_time_ = raw_start_time_;
    scan_end_time_ = raw_end_time_;
  }
  if (!buffer_cloud_->empty()) scan_start_time_ = buffer_start_time_;
  if (imu_end_time_ > scan_end_time_ && imu_start_time_ < scan_start_time_) {
    *scan_cloud_ = *buffer_cloud_;
    *scan_cloud_ += *raw_cloud_;
    scan_cloud_bins_ = buffer_cloud_bins_ + raw_cloud_bins_;
    buffer_cloud_->clear();
    buffer_cloud_bins_.setZero();
    buffer_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    buffer_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  } else {
    if (imu_end_time_ < scan_end_time_) {
      imu_time_offset_ = (scan_end_time_ - imu_end_time_).seconds();
      buffer_end_time_ = scan_end_time_;
      buffer_start_time_ = imu_end_time_;
      scan_end_time_ = imu_end_time_;
    }
    if (imu_start_time_ > scan_start_time_) scan_start_time_ = imu_start_time_;

    std::fill(scan_bin_sizes_.begin(), scan_bin_sizes_.end(), 0);
    std::fill(filter_bin_sizes_.begin(), filter_bin_sizes_.end(), 0);
    total_size = buffer_cloud_->size() + raw_cloud_->size();

    scan_cloud_->resize(total_size);
    filter_cloud_->resize(total_size);

#pragma omp parallel for
    for (int i = 0; i < total_size; i++) {
      const EllipseLioPoint& pt =
          i < buffer_cloud_->size()
              ? buffer_cloud_->points[i]
              : raw_cloud_->points[i - buffer_cloud_->size()];
      rclcpp::Time pt_time =
          rclcpp::Time(pt.time_secs, pt.time_nsecs, RCL_ROS_TIME);
      if (pt_time < scan_start_time_) continue;
      if (pt_time > scan_end_time_) {
        filter_bin_sizes_[pt.bin_idx]++;
        filter_cloud_->points[filter_idx++] = pt;
      } else {
        scan_bin_sizes_[pt.bin_idx]++;
        scan_cloud_->points[scan_idx++] = pt;
      }
    }

    scan_cloud_->resize(scan_idx);
    filter_cloud_->resize(filter_idx);
    *buffer_cloud_ = *filter_cloud_;

#pragma omp parallel for
    for (int i = 0; i < scan_bin_sizes_.size(); i++) {
      scan_cloud_bins_[i] = scan_bin_sizes_[i];
      buffer_cloud_bins_[i] = filter_bin_sizes_[i];
    }
  }

  raw_cloud_->clear();
  raw_cloud_bins_.setZero();
  raw_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  raw_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  if (!ekfom_data_sb_.sum()) {
    ekfom_data_sb_ += fmin(fmax(start_bin_ / 10.0, 1e-4), 1.0);
  } else {
    ekfom_data_sb_[start_bin_cnt_] = fmin(fmax(start_bin_ / 10.0, 1e-4), 1.0);
  }
  start_bin_cnt_ = (start_bin_cnt_ + 1) % ekfom_data_sb_.size();

  analytics_msg_.imu_offset = imu_time_offset_;
  analytics_msg_.lid_offset = lid_time_offset_;
  analytics_msg_.scan_size = scan_cloud_->size();
  analytics_msg_.buffer_size = buffer_cloud_->size();
  analytics_msg_.scan_time = (scan_end_time_ - scan_start_time_).seconds();
}

void MappingNode::ComputeRamUsage() {
  double vm_usage = 0.0;
  double resident_set = 0.0;
  std::ifstream stat_stream("/proc/self/stat", std::ios_base::in);
  std::string pid, comm, state, ppid, pgrp, session, tty_nr;
  std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
  std::string utime, stime, cutime, cstime, priority, nice;
  std::string num_threads, itrealvalue, starttime;
  unsigned long vsize;
  long rss;
  stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr >>
      tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt >> utime >>
      stime >> cutime >> cstime >> priority >> nice >> num_threads >>
      itrealvalue >> starttime >> vsize >> rss;
  stat_stream.close();
  long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024;
  vm_usage = vsize / 1024.0;
  resident_set = rss * page_size_kb;

  analytics_msg_.ram_usage = round(resident_set / 1000.0);
}

void MappingNode::ComputeCpuUsage() {
  struct tms timeSample;
  clock_t now;
  double cpu_percent;
  now = times(&timeSample);
  if (now <= last_cpu_ || timeSample.tms_stime < last_sys_cpu_ ||
      timeSample.tms_utime < last_user_cpu_) {
    cpu_percent = -1.0;
  } else {
    cpu_percent = (timeSample.tms_stime - last_sys_cpu_) +
                  (timeSample.tms_utime - last_user_cpu_);
    cpu_percent /= (now - last_cpu_);
    cpu_percent /= num_processors_;
    cpu_percent *= 100.;
  }
  last_cpu_ = now;
  last_sys_cpu_ = timeSample.tms_stime;
  last_user_cpu_ = timeSample.tms_utime;

  analytics_msg_.cpu_usage = round(cpu_percent);
}

void MappingNode::TimerCallback() {
  if (!initialized_) {
    initialized_ = true;
    imu_process_ =
        std::make_shared<ImuProcess>(kf_, imu_params_, shared_from_this());
    lid_process_ = std::make_shared<LidarProcess>(
        lidar_params_, map_resolution_, shared_from_this());
    InitCamProcess();

    n_res_ = Eigen::ArrayXf::Ones(lidar_params_.rate);
    n_res_ *= 0.5;
    n_means_ = Eigen::ArrayXi::Zero(lid_process_->num_bins_);
    n_cnts_ = Eigen::ArrayXXi::Zero(kMaxProcPoints, lid_process_->num_bins_);
    n_bins_ = Eigen::ArrayXXi::Zero(kMaxProcPoints, lid_process_->num_bins_);

    raw_cloud_bins_ = Eigen::ArrayXi::Zero(lid_process_->num_bins_);
    scan_cloud_bins_ = Eigen::ArrayXi::Zero(lid_process_->num_bins_);
    buffer_cloud_bins_ = Eigen::ArrayXi::Zero(lid_process_->num_bins_);
    scan_bin_sizes_ = std::vector<std::atomic<int>>(lid_process_->num_bins_);
    filter_bin_sizes_ = std::vector<std::atomic<int>>(lid_process_->num_bins_);
  }

  if (SyncPackages()) {
    double t1, t2, t3, t4, imu_time, state_time, map_time, total_time;

    t1 = omp_get_wtime();

    imu_process_->UndistortPointCloud(scan_cloud_, &kf_state_, scan_start_time_,
                                      scan_end_time_, cams_process_);

    t2 = omp_get_wtime();

    if (map_counter_) {
      ekfom_iter_cnt_ = 0;
      imu_process_->UpdateStatesWithLidar(&kf_state_, scan_end_time_,
                                          0.5 / lidar_params_.rate);
    }

    t3 = omp_get_wtime();

    map_mutex_.lock();
    MapIncremental();
    map_mutex_.unlock();

    t4 = omp_get_wtime();

    imu_time = t2 - t1;
    state_time = t3 - t2;
    map_time = t4 - t3;
    total_time = t4 - t1;

    max_imu_time_ = fmax(max_imu_time_, imu_time);
    max_state_time_ = fmax(max_state_time_, state_time);
    max_map_time_ = fmax(max_map_time_, map_time);
    max_total_time_ = fmax(max_total_time_, total_time);

    mean_imu_time_ += imu_time;
    mean_state_time_ += state_time;
    mean_map_time_ += map_time;
    mean_total_time_ += total_time;

    ComputeRamUsage();
    ComputeCpuUsage();

    analytics_msg_.run_time = round(t4 - start_time_);

    analytics_msg_.imu_time = imu_time;
    analytics_msg_.state_time = state_time;
    analytics_msg_.map_time = map_time;
    analytics_msg_.total_time = total_time;

    analytics_msg_.imu_mean = mean_imu_time_ / (map_counter_ + 1);
    analytics_msg_.state_mean = mean_state_time_ / (map_counter_ + 1);
    analytics_msg_.map_mean = mean_map_time_ / (map_counter_ + 1);
    analytics_msg_.total_mean = mean_total_time_ / (map_counter_ + 1);

    analytics_msg_.imu_max = max_imu_time_;
    analytics_msg_.state_max = max_state_time_;
    analytics_msg_.map_max = max_map_time_;
    analytics_msg_.total_max = max_total_time_;

    odom_mutex_.lock();
    kf_state_pub_ = kf_state_;
    *scan_cloud_pub_ = *scan_cloud_;
    analytics_msg_pub_ = analytics_msg_;
    odom_mutex_.unlock();

    map_counter_++;
  }
}

}  // namespace ellipselio

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ellipselio::MappingNode)