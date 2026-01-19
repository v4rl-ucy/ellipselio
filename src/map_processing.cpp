#include <map_processing.h>

namespace ellipselio {

// Sync lidar, imu, and camera data
bool MappingNode::sync_packages() {
  double velocity;
  bool got_lidar_data;
  KfState latest_state;

  auto& clk = *this->get_clock();
  double lidar_scan_time = 1.0 / lidar_params.rate;
  double inter_sync_time = omp_get_wtime() - last_sync_time;
  rclcpp::Duration lidar_scan_duration(0, 1e9 * lidar_scan_time);

  if (!last_sync_time) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), clk, 1000, "Waiting for data...");
  }

  if (imu_process->lidar_ready_ &&
      imu_process->imu_end_time_ <= last_imu_time_) {
    if (inter_sync_time > 1.0) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), clk, 1000,
                            "IMU has no new data");
    }
    return false;
  }
  publish_imu_odometry();

  if (!lid_process->lidar_has_data_ && buffer_cloud->empty() &&
      raw_cloud->empty()) {
    if (inter_sync_time > 1.0) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), clk, 1000,
                            "Lidar has no new data");
    }
    return false;
  }
  imu_process->lidar_ready_ = true;

  if (lid_process->lidar_start_time_ < imu_process->imu_start_time_) {
    lid_process->ClearPointCloud();
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), clk, 1000,
                                 "Lidar start time is before IMU start time");
    return false;
  }

  got_lidar_data =
      lid_process->GetPointCloud(raw_cloud, raw_start_time_, raw_end_time_,
                                 raw_cloud_bins, start_bin, mean_bin);

  if (raw_cloud->empty() && buffer_cloud->empty()) {
    if (inter_sync_time > lidar_scan_time) {
      RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), clk, 1000,
                                   "No synced measurements");
    }
    return false;
  }

  for (int i = 0; i < num_cams; i++) {
    if (!cams_process[i]->cam_has_data_) {
      if (inter_sync_time > 1.0) {
        RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), clk, 1000,
                                     "Camera " << i << " has no new data");
      }
    }
  }

  imu_start_time_ = imu_process->imu_start_time_;
  imu_end_time_ = imu_process->imu_end_time_;
  last_imu_time_ = imu_process->imu_end_time_;

  scan_start_time_ = buffer_start_time_;
  scan_end_time_ = raw_end_time_;

  if (buffer_cloud->empty()) {
    scan_start_time_ = raw_start_time_;
  }
  if (raw_cloud->empty()) {
    scan_end_time_ = buffer_end_time_;
  }
  if (imu_start_time_ > scan_start_time_) {
    scan_start_time_ = imu_start_time_;
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "IMU start time later than scan start time");
    buffer_cloud->clear();
    return false;
  }
  if (scan_end_time_ - scan_start_time_ < lidar_scan_duration) {
    if (!buffer_cloud->empty() || scan_end_time_ > imu_end_time_) {
      return false;
    }
  } else {
    if (scan_start_time_ + lidar_scan_duration > imu_end_time_) {
      return false;
    }
  }

  sync_raw_cloud_with_imu();

  int cur_imu_freq = round(imu_process->imu_counter_ / inter_sync_time);
  int cur_lid_freq = round(lid_process->lidar_counter_ / inter_sync_time);
  imu_process->imu_counter_ = 0;
  lid_process->lidar_counter_ = 0;

  analytics_msg_.imu_freq = cur_imu_freq;
  analytics_msg_.lid_freq = cur_lid_freq;

  analytics_msg_.cams_freq.clear();
  for (int i = 0; i < num_cams; i++) {
    int cur_cam_freq = round(cams_process[i]->cam_counter_ / inter_sync_time);
    cams_process[i]->cam_counter_ = 0;
    analytics_msg_.cams_freq.push_back(cur_cam_freq);
  }

  int cur_odom_freq = round(1.0 / inter_sync_time);
  analytics_msg_.odom_freq = cur_odom_freq;

  last_sync_time = omp_get_wtime();
  return true;
}

// Compute tensor voting matrix for point i and j
void MappingNode::compute_tensor_vote(int i, int j, M3F& A_j, bool first_pass) {
  V3F p_i = map_cloud->points[i].getVector3fMap();
  V3F p_j = map_cloud->points[j].getVector3fMap();
  const int& bin_idx = map_cloud->points[i].bin_idx;
  float search_rad = lid_process->search_radii_[bin_idx];
  float d_ij = (p_i - p_j).norm();
  float c_ij = std::exp(-std::pow(d_ij, 2) / search_rad);
  V3F r_ij = (p_i - p_j).normalized();
  M3F rrt = r_ij * r_ij.transpose();
  M3F R_ij = Eye3f - 2.0 * rrt;
  M3F Rp_ij = (Eye3f - 0.5 * rrt) * R_ij;
  M3F K_j = Eye3f;
  if (!first_pass) K_j = tensors_p2[j];
  A_j = c_ij * R_ij * K_j * Rp_ij;
}

// Compute tensor eigenvalues and eigenvectors for point i
void MappingNode::compute_tensor_eigen(int i, M3F& tensor, bool first_pass) {
  V3F eig_val, sali_val;
  M3F eig_vec, tensor_i2;
  Eigen::SelfAdjointEigenSolver<M3F> eig_solver;

  eig_solver.computeDirect(tensor);
  eig_vec = eig_solver.eigenvectors();
  eig_val = eig_solver.eigenvalues().cwiseAbs();

  const int& bin_idx = map_cloud->points[i].bin_idx;
  float search_rad = lid_process->search_radii_[bin_idx];

  if (first_pass) {
    tensor_i2 =
        (eig_val(2) - eig_val(1)) * eig_vec.col(2) * eig_vec.col(2).transpose();
    tensor_i2 += (eig_val(1) - eig_val(0)) *
                 (eig_vec.col(2) * eig_vec.col(2).transpose() +
                  eig_vec.col(1) * eig_vec.col(1).transpose());
    tensors_p2[i] = tensor_i2;
  } else {
    sali_val(0) = eig_val(2) - eig_val(1);
    sali_val(1) = eig_val(1) - eig_val(0);
    sali_val(2) = eig_val(0);
    sali_val.maxCoeff(&saliency_idxs[i]);

    filters[i][1] = true;
    salivalues[i] = sali_val;
    eigenvalues[i] = (1.0 / (eig_val.array() + 1e-10)).matrix().normalized();
    eigenvalues[i] *= search_rad;
    eigenvectors[i] = eig_vec;
    map_cloud->points[i].prim_type = (saliency_idxs[i] + 1) * 85;
    if (num_cams) return;
    switch (saliency_idxs[i]) {
      case 0:
        map_cloud->points[i].r = 32;
        map_cloud->points[i].g = 144;
        map_cloud->points[i].b = 240;
        break;
      case 1:
        map_cloud->points[i].r = 94;
        map_cloud->points[i].g = 201;
        map_cloud->points[i].b = 98;
        break;
      case 2:
        map_cloud->points[i].r = 253;
        map_cloud->points[i].g = 231;
        map_cloud->points[i].b = 36;
        break;
    }
  }
}

// Compute first pass tensor voting for new points and find neighbours
void MappingNode::tensor_vote_pass_1(int old_map_size,
                                     std::vector<int>& added_idxs,
                                     std::vector<int>& updated_idxs) {
  int added_size = added_idxs.size();
  std::atomic<int> upd_idx = 0, new_neighbours_idx = 0;

#pragma omp parallel for
  for (int i = 0; i < added_size; i++) {
    int map_i;

    map_i = added_idxs[i];
    valid_reg[map_i] = 1;
    updated_pt[map_i] = 0;
    map_cloud->points[map_i].prim_type = 0;
    colors[map_i] =
        map_cloud->points[map_i].getRGBVector3i().cast<float>() / 255.0f;

    const int& bin_idx = map_cloud->points[map_i].bin_idx;
    const int& bucket_size = lid_process->bucket_sizes_[bin_idx];
    const float search_rad = lid_process->search_radii_[bin_idx];

    neighbours[map_i].reserve(lid_process->max_neighbours_[bin_idx]);
    ioctree.radiusNeighbors(map_cloud->points[map_i], search_rad,
                            neighbours[map_i], bucket_size);

    n_bins.row(i).setZero();
    n_cnts.row(i).setZero();
    n_bins(i, bin_idx) = 1;
    n_cnts(i, bin_idx) = neighbours[map_i].size();
  }

#pragma omp parallel for
  for (int i = 0; i < lid_process->num_bins_; i++) {
    int n_bins_sum = n_bins.col(i).head(added_size).sum();
    int tot_sum = lid_process->cnt_neighbours_[i] + n_bins_sum;
    if (!n_bins_sum) continue;
    n_means(i) = floor(n_means(i) * lid_process->cnt_neighbours_[i]);
    n_means(i) += n_cnts.col(i).head(added_size).sum();
    n_means(i) = floor(n_means(i) / tot_sum);
    n_means(i) = fmin(fmax(n_means(i), MIN_NEIGHBOURS), MAX_NEIGHBOURS);
    lid_process->min_neighbours_[i] = n_means(i);
    lid_process->max_neighbours_[i] = fmin(2 * n_means(i), MAX_NEIGHBOURS);
    lid_process->cnt_neighbours_[i] += n_bins_sum;
  }

#pragma omp parallel for
  for (int i = 0; i < added_idxs.size(); i++) {
    int map_i, loop_cnt;
    Eigen::MatrixXf K;
    M3F tensor_i1;

    map_i = added_idxs[i];

    const int& bin_idx = map_cloud->points[map_i].bin_idx;
    const int& min_neigh = lid_process->min_neighbours_[bin_idx];
    const int& max_neigh = lid_process->max_neighbours_[bin_idx];

    loop_cnt = min(int(neighbours[map_i].size()), max_neigh);
    K = Eigen::MatrixXf::Zero(loop_cnt, 9);

#pragma omp parallel for
    for (int j = 0; j < loop_cnt; j++) {
      M3F A_j;
      int map_j = neighbours[map_i][j];
      compute_tensor_vote(map_i, map_j, A_j, true);
      K.row(j) = A_j.reshaped(1, 9);

      if (map_j >= old_map_size) continue;

      const int& bin_idx_j = map_cloud->points[map_j].bin_idx;
      float search_rad_j = lid_process->search_radii_[bin_idx_j];
      EllipseLioPoint& pt_i = map_cloud->points[map_i];
      EllipseLioPoint& pt_j = map_cloud->points[map_j];
      float d_ij = (pt_i.getVector3fMap() - pt_j.getVector3fMap()).norm();

      if (d_ij > search_rad_j) continue;

      if (!(updated_pt[map_j]++)) {
        update_idx[map_j] = new_neighbours_idx++;
        new_neighbours_map_idx[update_idx[map_j]] = map_j;
        new_neighbours_size[update_idx[map_j]] = 0;
        new_neighbours[update_idx[map_j]]
                      [new_neighbours_size[update_idx[map_j]]++] = map_i;
      } else if (new_neighbours_size[update_idx[map_j]] < MAX_NEIGHBOURS) {
        new_neighbours[update_idx[map_j]]
                      [new_neighbours_size[update_idx[map_j]]++] = map_i;
      }
    }
    tensors_p1[map_i] = K.colwise().sum().reshaped(3, 3);

    filters[map_i][0] = loop_cnt >= min_neigh;
    if (!filters[map_i][0]) continue;

    tensor_i1 = tensors_p1[map_i] / float(loop_cnt);
    compute_tensor_eigen(map_i, tensor_i1, true);
  }

  updated_idxs.resize(new_neighbours_idx);

#pragma omp parallel for
  for (int i = 0; i < new_neighbours_idx; i++) {
    Eigen::MatrixXf K;
    M3F tensor_i1;
    int map_i, loop_cnt, max_loop, old_size;

    map_i = new_neighbours_map_idx[i];
    updated_pt[map_i] = 0;

    const int& bin_idx = map_cloud->points[map_i].bin_idx;
    const int& min_neigh = lid_process->min_neighbours_[bin_idx];
    const int& max_neigh = lid_process->max_neighbours_[bin_idx];
    if (neighbours[map_i].size() >= max_neigh) continue;

    updated_idxs[upd_idx++] = map_i;

    max_loop = max_neigh - neighbours[map_i].size();
    loop_cnt = min(int(new_neighbours_size[i]), max_loop);

    old_size = neighbours[map_i].size();
    neighbours[map_i].resize(old_size + loop_cnt);

    K = Eigen::MatrixXf::Zero(loop_cnt, 9);

#pragma omp parallel for
    for (int j = 0; j < loop_cnt; j++) {
      M3F A_j;
      int map_j = new_neighbours[i][j];
      neighbours[map_i][old_size + j] = map_j;
      compute_tensor_vote(map_i, map_j, A_j, true);
      K.row(j) = A_j.reshaped(1, 9);
    }

    tensors_p1[map_i] += K.colwise().sum().reshaped(3, 3);

    filters[map_i][0] = neighbours[map_i].size() >= min_neigh;
    if (!filters[map_i][0]) continue;

    tensor_i1 = tensors_p1[map_i] / float(neighbours[map_i].size());
    compute_tensor_eigen(map_i, tensor_i1, true);
  }
  updated_idxs.resize(upd_idx);
}

// Compute second pass tensor voting for new and existing points
void MappingNode::tensor_vote_pass_2(std::vector<int>& added_idxs,
                                     std::vector<int>& updated_idxs) {
  int total_size = added_idxs.size() + updated_idxs.size();

#pragma omp parallel for
  for (int i = 0; i < total_size; i++) {
    SHCoeffs SH;
    M3F tensor_i2;
    Eigen::MatrixXf K;
    V3F sh_dir, sh_color;
    Eigen::VectorXi K_filter, SH_filter;
    int map_i, loop_cnt, filter_cnt, color_cnt;

    map_i = i < added_idxs.size() ? added_idxs[i]
                                  : updated_idxs[i - added_idxs.size()];
    valid_reg[map_i] = 1;

    const int& bin_idx = map_cloud->points[map_i].bin_idx;
    const int& min_neigh = lid_process->min_neighbours_[bin_idx];
    const int& max_neigh = lid_process->max_neighbours_[bin_idx];
    loop_cnt = min(int(neighbours[map_i].size()), max_neigh);

    if (!filters[map_i][0]) continue;

    K = Eigen::MatrixXf::Zero(loop_cnt, 9);
    K_filter = Eigen::VectorXi::Zero(loop_cnt);

    // if (num_cams) {
    //   SH_filter = Eigen::VectorXi::Zero(loop_cnt + 1);
    //   SH = SHCoeffs(loop_cnt + 1, harmonics->getNumCoeffs());
    // }
    // if (map_cloud->points[map_i].has_rgb) {
    //   SH_filter(loop_cnt) = 1;
    //   compute_harmonics(map_i, map_i, loop_cnt, SH);
    // }

#pragma omp parallel for
    for (int j = 0; j < loop_cnt; j++) {
      int map_j = neighbours[map_i][j];
      if (!filters[map_j][0]) continue;

      M3F A_j;
      compute_tensor_vote(map_i, map_j, A_j, false);
      K.row(j) = A_j.reshaped(1, 9);
      K_filter(j) = 1;

      // if (map_cloud->points[map_j].has_rgb) {
      //   SH_filter(j) = 1;
      //   compute_harmonics(map_i, map_j, j, SH);
      // }
    }

    filter_cnt = K_filter.sum();
    if (filter_cnt < min_neigh) continue;

    tensor_i2 = K.colwise().sum().reshaped(3, 3);
    tensor_i2 /= float(filter_cnt);
    compute_tensor_eigen(map_i, tensor_i2, false);

    // if (num_cams) {
    //   color_cnt = SH_filter.sum();
    //   if (color_cnt < min_neigh) continue;

    //   sh_dir = poses[map_cloud->points[map_i].scan_idx];
    //   sh_dir -= map_cloud->points[map_i].getVector3fMap();
    //   harmonics->finalizeCoefficients(SH, sh_mats[map_i]);
    //   harmonics->evaluateColorFromDirection(sh_mats[map_i], sh_dir,
    //   sh_color); map_cloud->points[map_i].r = sh_color(0) * 255.0f;
    //   map_cloud->points[map_i].g = sh_color(1) * 255.0f;
    //   map_cloud->points[map_i].b = sh_color(2) * 255.0f;
    // }
  }
}

void MappingNode::compute_harmonics(int map_i, int map_j, int loop_idx,
                                    SHCoeffs& SH) {
  Eigen::Vector3f dir;

  const int& bin_idx = map_cloud->points[map_i].bin_idx;
  const float& search_rad = bin_idx;
  const Eigen::Vector3f& pose = poses[map_cloud->points[map_j].scan_idx];
  const Eigen::Vector3f& p_i = map_cloud->points[map_i].getVector3fMap();
  const Eigen::Vector3f& p_j = map_cloud->points[map_j].getVector3fMap();

  harmonics->dirFromNeighbouringPoint(p_i, p_j, pose, dir, search_rad);
  harmonics->computeCoefficients(dir, colors[map_j], SH, loop_idx);
}

// Add new points to the map and update geometric primitives
void MappingNode::map_incremental() {
  int start_idx, end_idx;
  V3F grav_norm, axis_norm;
  std::vector<int> new_idxs, updated_idxs, added_idxs, map_idxs;

  poses.push_back(kf_state_.state.pos.cast<float>());
  rotes.push_back(kf_state_.state.rot.cast<float>());

  if (kf_state_.state.vel.norm() > 0.5 || vel_poses.empty()) {
    vel_pose_counter++;
    curr_vel_streak--;
    curr_vel_streak = fmax(curr_vel_streak, 0);
    vel_poses.push_back(kf_state_.state.pos.cast<float>());
  } else {
    curr_vel_streak++;
  }

  if (poses.back().norm() > mean_bin && scale_search) {
    scale_search = false;
  }

  if (!map_counter) {
    grav_norm = kf_state_.state.grav.get_vect().normalized().cast<float>();
    axis_norm = kf_state_.state.rot.cast<float>() * Eigen::Vector3f::UnitZ();
    axis_grav_align = fabs(grav_norm.dot(axis_norm)) > 0.8;
  }

#pragma omp parallel for
  for (int i = 0; i < scan_cloud->size(); i++) {
    scan_cloud->points[i].scan_idx = map_counter;
    const int& bin_idx = scan_cloud->points[i].bin_idx;
    scan_cloud->points[i].bin_idx = fmax(bin_idx, start_bin);
    scan_cloud->points[i].getVector3fMap() =
        (kf_state_.state.rot *
             (kf_state_.state.offset_R_L_I *
                  scan_cloud->points[i].getVector3fMap().cast<double>() +
              kf_state_.state.offset_T_L_I) +
         kf_state_.state.pos)
            .cast<float>();
  }

  start_idx = 0;
  end_idx = 0;
  old_map_size = map_cloud->size();

  for (int i = 0; i < scan_cloud_bins.size(); i++) {
    end_idx += scan_cloud_bins[i];
    if (!scan_cloud_bins[i]) continue;
    if (end_idx > scan_cloud->size()) break;

    if (!init_poses[i]) {
      last_updated_poses[i] = poses[map_counter];
      last_updated_rotes[i] = rotes[map_counter];
    }

    const float& min_oct_res = lid_process->octree_resolutions_.front();
    float pose_diff = (poses[map_counter] - last_updated_poses[i]).norm();
    bool add_check = pose_diff < min_oct_res && mean_bin > start_bin;
    if (add_check && scale_search && init_poses[i]) continue;

    last_updated_poses[i] = poses[map_counter];
    last_updated_rotes[i] = rotes[map_counter];

    ioctree.set_bucket_size(lid_process->bucket_sizes_[fmax(i, start_bin)]);
    ioctree.update(*scan_cloud, added_idxs, map_idxs, start_idx, end_idx,
                   map_resolution);
    *map_cloud += EllipseLioPointCloud(*scan_cloud, added_idxs);
    new_idxs.insert(new_idxs.end(), map_idxs.begin(), map_idxs.end());
    start_idx = end_idx;

    if (!init_poses[i]) init_poses[i] = true;
  }

  new_map_size = map_cloud->size();

  valid_reg.resize(map_cloud->size(), 0);
  update_idx.resize(map_cloud->size(), 0);
  saliency_idxs.resize(map_cloud->size(), 0);
  neighbours.resize(map_cloud->size(), std::vector<int>());
  colors.resize(map_cloud->size(), Eigen::Vector3f::Zero());
  filters.resize(map_cloud->size(), Eigen::Vector2i::Zero());

  tensors_p1.resize(map_cloud->size(), M3F::Zero());
  tensors_p2.resize(map_cloud->size(), M3F::Zero());
  salivalues.resize(map_cloud->size(), V3F::Zero());
  eigenvalues.resize(map_cloud->size(), V3F::Zero());
  eigenvectors.resize(map_cloud->size(), M3F::Zero());

  if (new_idxs.size() > 0) {
    tensor_vote_pass_1(old_map_size, new_idxs, updated_idxs);
    tensor_vote_pass_2(new_idxs, updated_idxs);
  }

  analytics_msg_.map_size = map_cloud->size();
  analytics_msg_.oct_num = ioctree.octant_size();
  analytics_msg_.new_idxs = new_idxs.size();
  analytics_msg_.upd_idxs = updated_idxs.size();
}

void MappingNode::split_map(const sensor_msgs::msg::PointCloud2& input,
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

// Publish map point cloud
void MappingNode::publish_map() {
  if (!map_counter) return;

  sensor_msgs::msg::PointCloud2 map_msg;
  std::vector<sensor_msgs::msg::PointCloud2> map_parts;
  if (!map_cloud->size()) return;

  map_mutex_.lock();
  publish_markers();
  pcl::toROSMsg(*map_cloud, map_msg);
  map_mutex_.unlock();

  map_msg.header.stamp = kf_state_pub_.time;
  map_msg.header.frame_id = "odom_ellipselio";

  split_map(map_msg, map_parts, (1000 * pub_map_n_secs) / 100);
  for (auto& part : map_parts) {
    pub_map_->publish(part);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
}

// Publish scan point cloud
void MappingNode::publish_scan() {
  if (!map_counter) return;

  sensor_msgs::msg::PointCloud2 scan_msg;
  pcl::toROSMsg(*scan_cloud_pub, scan_msg);
  scan_msg.header.stamp = kf_state_pub_.time;
  scan_msg.header.frame_id = "odom_ellipselio";
  pub_scan_->publish(scan_msg);
}

// Publish geometric primitive markers
void MappingNode::publish_markers() {
  if (!map_counter) return;

  std::atomic<int> marker_idx = 0;
  visualization_msgs::msg::MarkerArray marker_array;

  if (!map_cloud->size()) return;

  int count_idx = std::ceil(0.01 * (new_map_size - last_map_size));

  marker_array.markers.resize(count_idx);
#pragma omp parallel for
  for (int i = last_map_size; i < new_map_size; i += 100) {
    Eigen::Quaternionf quat;
    visualization_msgs::msg::Marker marker;

    int map_idx = i;
    if (!filters[map_idx][1]) continue;

    marker.id = map_idx;
    marker.frame_locked = true;
    marker.lifetime = rclcpp::Duration(0, 0);
    marker.header.frame_id = "odom_ellipselio";
    marker.header.stamp = kf_state_pub_.time;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    quat = eigenvectors[map_idx];

    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();

    marker.pose.position.x = map_cloud->points[map_idx].x;
    marker.pose.position.y = map_cloud->points[map_idx].y;
    marker.pose.position.z = map_cloud->points[map_idx].z;

    switch (saliency_idxs[map_idx]) {
      case 0:
        marker.ns = "plane";
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = 2 * eigenvalues[map_idx](0);
        marker.scale.y = 2 * eigenvalues[map_idx](1);
        marker.scale.z = 2 * eigenvalues[map_idx](2);
        marker.color.r = 32.0 / 255.0;
        marker.color.g = 144.0 / 255.0;
        marker.color.b = 240.0 / 255.0;
        break;
      case 1:
        marker.ns = "line";
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = 2 * eigenvalues[map_idx](0);
        marker.scale.y = 2 * eigenvalues[map_idx](1);
        marker.scale.z = 2 * eigenvalues[map_idx](2);
        marker.color.r = 94.0 / 255.0;
        marker.color.g = 201.0 / 255.0;
        marker.color.b = 98.0 / 255.0;
        break;
      case 2:
        marker.ns = "ball";
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = 2 * eigenvalues[map_idx](0);
        marker.scale.y = 2 * eigenvalues[map_idx](1);
        marker.scale.z = 2 * eigenvalues[map_idx](2);
        marker.color.r = 253.0 / 255.0;
        marker.color.g = 231.0 / 255.0;
        marker.color.b = 36.0 / 255.0;
        break;
    }
    marker_array.markers[marker_idx++] = marker;
  }
  last_map_size = new_map_size;
  marker_array.markers.resize(marker_idx);
  pub_mark_->publish(marker_array);
}

// Publish odometry transform
void MappingNode::publish_imu_odometry() {
  KfState imu_state;

  if (!map_counter) return;

  imu_process->GetKfState(imu_state);
  if (last_imu_pub_time == imu_state.time) return;
  last_imu_pub_time = imu_state.time;

  geometry_msgs::msg::TransformStamped trans;
  trans.header.frame_id = "odom_ellipselio";
  trans.child_frame_id = "raw_ellipselio";
  trans.header.stamp = imu_state.time;
  trans.transform.translation.x = imu_state.state.pos(0);
  trans.transform.translation.y = imu_state.state.pos(1);
  trans.transform.translation.z = imu_state.state.pos(2);
  trans.transform.rotation.x = imu_state.state.rot.coeffs()[0];
  trans.transform.rotation.y = imu_state.state.rot.coeffs()[1];
  trans.transform.rotation.z = imu_state.state.rot.coeffs()[2];
  trans.transform.rotation.w = imu_state.state.rot.coeffs()[3];
  tf_br_->sendTransform(trans);
}

// Publish odometry transform
void MappingNode::publish_opt_odometry() {
  if (!map_counter) return;

  if (last_opt_pub_time == kf_state_pub_.time) return;
  odom_mutex_.lock();
  last_opt_pub_time = kf_state_pub_.time;

  geometry_msgs::msg::TransformStamped trans;
  trans.header.frame_id = "odom_ellipselio";
  trans.child_frame_id = "imu_ellipselio";
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
  publish_scan();
  odom_mutex_.unlock();
}

// Register new scan points to the map using tensor registration
void MappingNode::tensor_registration(
    state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) {
  double t0, t1, res_mean;
  int feat_tot, reject_cnt;
  float wt_min, wt_max, wt_mean, wt_std, obs_min, rng_scale;
  float rng_min, rng_max, rng_mean, rng_min_scale, rng_max_scale, grav_check;

  std::atomic<int> feat_cnt;
  std::vector<std::atomic<int>> prim_cnts(3);

  Eigen::Array3i cnts(3);
  Eigen::Array3d rot_obs, tran_obs;
  V3F grav_norm, poses_diff;

  feat_cnt = 0;
  prim_cnts[0] = 0;
  prim_cnts[1] = 0;
  prim_cnts[2] = 0;

  grav_norm = kf_state_.state.grav.get_vect().normalized().cast<float>();
  poses_diff = s.pos.cast<float>();
  poses_diff -= vel_poses[fmax(vel_pose_counter - 10, 0)];
  grav_check = fabs(grav_norm.dot(poses_diff));
  grav_check *= fabs(grav_norm.dot(poses_diff.normalized()));
  grav_check = fmin(grav_check, 1.0);

  t0 = omp_get_wtime();

#pragma omp parallel for
  for (int i = 0; i < scan_cloud->size(); i++) {
    Eigen::VectorXd h_x_vec(6);
    Eigen::VectorXd::Index tran_idx, rot_idx;
    std::vector<int> N_idxs;
    std::vector<float> N_dst;
    rclcpp::Time map_pt_time, scan_pt_time;
    int sali_idx, map_i, feat_num, prim_num;
    float bin_scale, search_rad, search_rad_scale, octree_res;
    float residual, time_score, time_pow, norm_check;

    M3D P_skew;
    V3D p_lidar, p_imu, a, obs_trans, obs_rot, obs_idx_trans, obs_idx_rot;
    V3F sali_vals, scores, p_world, n_world, p_dash, q, q_dash, norm_vec,
        a_world;

    const EllipseLioPoint& pt = scan_cloud->points[i];

    p_lidar = pt.getVector3fMap().cast<double>();
    p_imu = s.offset_R_L_I * p_lidar + s.offset_T_L_I;
    p_world = (s.rot * p_imu + s.pos).cast<float>();

    const int& bin_idx = scan_cloud->points[i].bin_idx;
    const float& oct_res = lid_process->octree_resolutions_[bin_idx];
    const float& min_oct_res = lid_process->octree_resolutions_.front();

    bin_scale = 10.0 / mean_bin;
    if (!axis_grav_align) bin_scale = 20.0;
    search_rad = lid_process->match_radii_[bin_idx];
    search_rad_scale = poses.back().norm() / (bin_scale * search_rad);
    octree_res = fmin(oct_res, MIN_SEARCH_RES);

    search_rad_scale = fmax(search_rad_scale, octree_res);
    search_rad /= ekfom_div_cnt + 1;
    if (scale_search) search_rad = fmin(search_rad, search_rad_scale);
    search_rad = fmax(fmin(search_rad, MAX_SEARCH_RES), min_oct_res);

    ioctree.knnNeighbors(p_world, 1, N_idxs, N_dst, search_rad);
    if (N_idxs.size() == 0) continue;

    map_i = N_idxs[0];
    if (!filters[map_i][1]) continue;
    if (!valid_reg[map_i]) continue;

    map_pt_time =
        rclcpp::Time(map_cloud->points[map_i].time_secs,
                     map_cloud->points[map_i].time_nsecs, RCL_ROS_TIME);
    scan_pt_time = rclcpp::Time(scan_cloud->points[i].time_secs,
                                scan_cloud->points[i].time_nsecs, RCL_ROS_TIME);

    sali_vals = salivalues[map_i] / salivalues[map_i].sum();

    // Point to plane
    scores(0) = sali_vals(0);
    //  Point to line
    scores(1) = sali_vals(1);
    //  Point to point
    scores(2) = sali_vals(2);
    scores /= scores.sum();

    n_world = map_cloud->points[map_i].getVector3fMap();
    q = p_world - n_world;

    // Point to plane
    q_dash = q.dot(eigenvectors[map_i].col(2)) * eigenvectors[map_i].col(2);
    p_dash = scores(0) * (p_world - q_dash);
    // Point to line
    q_dash = q.dot(eigenvectors[map_i].col(0)) * eigenvectors[map_i].col(0);
    p_dash += scores(1) * (n_world + q_dash);
    // Point to point
    p_dash += scores(2) * n_world;

    norm_vec = p_world - p_dash;
    residual = norm_vec.norm();
    norm_vec.normalize();

    norm_check = fmax(1.0 - fabs(grav_norm.dot(norm_vec)), 1e-4);
    time_score = 1.0 / ((scan_pt_time - map_pt_time).seconds() + 1.0);
    time_score *= norm_check;
    time_score = 1.0 / fmin(fmax(time_score, 1e-4), 1.0);

    P_skew << SKEW_SYM_MATRIX(p_imu);
    a = P_skew * s.rot.conjugate() * norm_vec.cast<double>();
    a_world = (s.rot * a).normalized().cast<float>();
    h_x_vec << norm_vec(0), norm_vec(1), norm_vec(2), a[0], a[1], a[2];

    obs_trans(0) = pow(scores(0), 2);
    obs_trans(0) *= fabs(norm_vec.dot(Eigen::Vector3f::UnitX()));
    obs_trans(1) = pow(scores(0), 2);
    obs_trans(1) *= fabs(norm_vec.dot(Eigen::Vector3f::UnitY()));
    obs_trans(2) = pow(scores(0), 2);
    obs_trans(2) *= fabs(norm_vec.dot(Eigen::Vector3f::UnitZ()));
    obs_rot(0) = fabs(a_world.dot(Eigen::Vector3f::UnitX()));
    obs_rot(1) = fabs(a_world.dot(Eigen::Vector3f::UnitY()));
    obs_rot(2) = fabs(a_world.dot(Eigen::Vector3f::UnitZ()));

    obs_trans.head(3).maxCoeff(&tran_idx);
    obs_rot.head(3).maxCoeff(&rot_idx);

    obs_idx_trans = Eigen::Vector3d::Zero(3);
    obs_idx_trans(tran_idx) = 1;
    obs_idx_rot = Eigen::Vector3d::Zero(3);
    obs_idx_rot(rot_idx) = 1;

    feat_num = ++feat_cnt;
    sali_idx = saliency_idxs[map_i];
    prim_num = ++prim_cnts[sali_idx];

    ekfom_data_ot.row(feat_num - 1) = obs_trans;
    ekfom_data_or.row(feat_num - 1) = obs_rot;
    ekfom_data_w.row(feat_num - 1) = time_score;

    ekfom_data_h_v(feat_num - 1) = -residual;
    ekfom_data_h_x_v.row(feat_num - 1) = h_x_vec;

    ekfom_data_oit.row(feat_num - 1) = obs_idx_trans;
    ekfom_data_oir.row(feat_num - 1) = obs_idx_rot;
  }

  cnts << prim_cnts[0].load(), prim_cnts[1].load(), prim_cnts[2].load();
  feat_tot = feat_cnt.load();
  reject_cnt = scan_cloud->size() - feat_tot;

  if (feat_tot < 50) {
    ekfom_data.valid = false;
    return;
  }

  wt_min = ekfom_data_w.head(feat_tot).minCoeff();
  wt_max = ekfom_data_w.head(feat_tot).maxCoeff();
  wt_mean = ekfom_data_w.head(feat_tot).mean();
  wt_std = (ekfom_data_w.head(feat_tot) - wt_mean).square().sum();
  wt_std = sqrt(wt_std / (feat_tot - 1));

  rng_min = wt_mean - wt_std;
  rng_min = fmax(rng_min, wt_min);
  rng_min_scale = rng_min / wt_min;
  rng_mean = wt_mean - rng_min;
  rng_max = wt_max - rng_min;
  rng_max = fmin(rng_mean + wt_std, rng_max);
  rng_max_scale = rng_max / ((wt_max * rng_min_scale) - rng_min);

  if (wt_std) {
    ekfom_data_w.head(feat_tot) *= rng_min_scale;
    ekfom_data_w.head(feat_tot) -= rng_min;
    ekfom_data_w.head(feat_tot) *= rng_max_scale;
    ekfom_data_w.head(feat_tot) += 1.0;
  }

  ekfom_data_ot.topRows(feat_tot) *= ekfom_data_oit.topRows(feat_tot);
  ekfom_data_ot.topRows(feat_tot) *=
      ekfom_data_w.head(feat_tot).replicate(1, 3);
  ekfom_data_or.topRows(feat_tot) *= ekfom_data_oir.topRows(feat_tot);
  ekfom_data_or.topRows(feat_tot) *=
      ekfom_data_w.head(feat_tot).replicate(1, 3);

  tran_obs = ekfom_data_ot.topRows(feat_tot).colwise().sum() + 1e-4;
  rot_obs = ekfom_data_or.topRows(feat_tot).colwise().sum() + 1e-4;

  tran_obs /= tran_obs.minCoeff();
  rot_obs /= rot_obs.minCoeff();
  tran_obs = tran_obs.inverse();
  rot_obs = rot_obs.inverse();

  ekfom_data_oit.topRows(feat_tot) *= tran_obs.transpose();
  ekfom_data_oir.topRows(feat_tot) *= rot_obs.transpose();

  Eigen::Vector3f cent_proj;
  Eigen::Vector4f scan_centroid;
  pcl::compute3DCentroid(*scan_cloud, scan_centroid);

  cent_proj = scan_centroid.head(3).dot(grav_norm) * grav_norm;
  cent_proj = scan_centroid.head(3) - cent_proj;
  rng_scale = ((1e4 - fmin(rng_max, 1e4)) / 1000.0) + 10.0;

  obs_min = rng_scale * fmin(rot_obs.minCoeff(), tran_obs.minCoeff());
  obs_min *= fmax(1.0 - fmin(1.0 * grav_check, 1.0), 1e-4);
  if (axis_grav_align) obs_min *= fmin(20.0 / pow(cent_proj.norm(), 2), 1.0);
  if (axis_grav_align) obs_min *= mean_bin / 10.0;
  obs_min = fmin(fmax(obs_min, 1e-4), 1.0);

  ekfom_data_om[ekfom_obs_cnt] = obs_min;
  ekfom_obs_cnt = (ekfom_obs_cnt + 1) % ekfom_data_om.size();
  obs_min = fmax(ekfom_data_om.mean(), 0.1);

  ekfom_data_w.head(feat_tot) = ekfom_data_w.head(feat_tot).pow(obs_min);
  ekfom_data_h.block(0, 0, feat_tot, 1) = ekfom_data_h_v.head(feat_tot);
  ekfom_data_w_x.block(0, 0, feat_tot, 1) = ekfom_data_w.head(feat_tot);
  ekfom_data_h_x.block(0, 0, feat_tot, 6) = ekfom_data_h_x_v.topRows(feat_tot);

  wt_min = ekfom_data_w.head(feat_tot).minCoeff();
  wt_max = ekfom_data_w.head(feat_tot).maxCoeff();
  wt_mean = ekfom_data_w.head(feat_tot).mean();
  wt_std = (ekfom_data_w.head(feat_tot) - wt_mean).square().sum();
  wt_std = sqrt(wt_std / (feat_tot - 1));

  rng_min = 1;
  rng_max += 1;
  rng_mean += 1;

  ekfom_data_h_x_R.leftCols(feat_tot) =
      (ekfom_data_h_x.topRows(feat_tot).array().colwise() *
       ekfom_data_w_x.head(feat_tot))
          .transpose();

  res_mean = -ekfom_data_h.head(feat_tot).sum() / feat_tot;
  ekfom_data.h = ekfom_data_h.head(feat_tot);
  ekfom_data.h_x = ekfom_data_h_x.topRows(feat_tot);
  ekfom_data.h_x_R = ekfom_data_h_x_R.leftCols(feat_tot);

  ekfom_iter_cnt++;
  if (feat_tot > 200) ekfom_div_cnt++;

  analytics_msg_.num_planes = cnts(0);
  analytics_msg_.num_lines = cnts(1);
  analytics_msg_.num_balls = cnts(2);
  analytics_msg_.wt_std = wt_std;
  analytics_msg_.wt_min = wt_min;
  analytics_msg_.wt_max = wt_max;
  analytics_msg_.wt_mean = wt_mean;
  analytics_msg_.rng_min = rng_min;
  analytics_msg_.rng_max = rng_max;
  analytics_msg_.rng_mean = rng_mean;
  analytics_msg_.res_mean = res_mean;
  analytics_msg_.mean_bin = mean_bin;
  analytics_msg_.start_bin = start_bin;
  analytics_msg_.num_feats = feat_tot;
  analytics_msg_.num_reject = reject_cnt;
  analytics_msg_.kf_iterations = ekfom_iter_cnt;
}

// Main mapping node
MappingNode::MappingNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("mapping_node", options),
      map_cloud(new EllipseLioPointCloud()),
      raw_cloud(new EllipseLioPointCloud()),
      scan_cloud(new EllipseLioPointCloud()),
      filter_cloud(new EllipseLioPointCloud()),
      buffer_cloud(new EllipseLioPointCloud()),
      scan_cloud_pub(new EllipseLioPointCloud()),
      harmonics(new EllipsoidHarmonics()),
      kf_(new Ikfom()) {
  this->declare_parameter<int>("mapping.pub_map_n_secs", 10);
  this->declare_parameter<double>("mapping.map_resolution", 0.1);

  this->declare_parameter<int>("imu.rate", 100);
  this->declare_parameter<double>("imu.gyr_noise", 0.1);
  this->declare_parameter<double>("imu.acc_noise", 0.1);
  this->declare_parameter<double>("imu.gyr_bias", 0.0001);
  this->declare_parameter<double>("imu.acc_bias", 0.0001);
  this->declare_parameter<string>("imu.topic", "");

  this->declare_parameter<int>("lidar.type", 0);
  this->declare_parameter<int>("lidar.rate", 10);
  this->declare_parameter<int>("lidar.scan_lines", 64);
  this->declare_parameter<double>("lidar.min_range", 1.0);
  this->declare_parameter<double>("lidar.max_range", 100.0);
  this->declare_parameter<double>("lidar.vertical_fov", 64.0);
  this->declare_parameter<string>("lidar.topic", "");

  this->declare_parameter<vector<double>>("lidar.t_imu_lidar",
                                          vector<double>());
  this->declare_parameter<vector<double>>("lidar.r_imu_lidar",
                                          vector<double>());

  this->declare_parameter<int>("cameras.num_cams", 0);
  this->declare_parameter<string>("cameras.transport", "raw");
  this->declare_parameter<vector<long int>>("cameras.frame_rates",
                                            vector<long int>());
  this->declare_parameter<vector<string>>("cameras.cam_topics",
                                          vector<string>());
  this->declare_parameter<vector<double>>("cameras.cam_intrinsics",
                                          vector<double>());
  this->declare_parameter<vector<double>>("cameras.t_cam_lidars",
                                          vector<double>());
  this->declare_parameter<vector<double>>("cameras.r_cam_lidars",
                                          vector<double>());

  this->get_parameter_or<int>("mapping.pub_map_n_secs", pub_map_n_secs, 1);
  this->get_parameter_or<double>("mapping.map_resolution", map_resolution, 0.1);

  this->get_parameter_or<int>("imu.rate", imu_params.rate, 100);
  this->get_parameter_or<double>("imu.gyr_noise", imu_params.gyr_noise, 0.1);
  this->get_parameter_or<double>("imu.acc_noise", imu_params.acc_noise, 0.1);
  this->get_parameter_or<double>("imu.gyr_bias", imu_params.gyr_bias, 0.0001);
  this->get_parameter_or<double>("imu.acc_bias", imu_params.acc_bias, 0.0001);
  this->get_parameter_or<string>("imu.topic", imu_params.topic, "");

  this->get_parameter_or<int>("lidar.type", lidar_params.type, 0);
  this->get_parameter_or<int>("lidar.rate", lidar_params.rate, 10);
  this->get_parameter_or<int>("lidar.scan_lines", lidar_params.scan_lines, 64);
  this->get_parameter_or<string>("lidar.topic", lidar_params.topic, "");
  this->get_parameter_or<double>("lidar.min_range", lidar_params.min_range,
                                 1.0);
  this->get_parameter_or<double>("lidar.max_range", lidar_params.max_range,
                                 100.0);
  this->get_parameter_or<double>("lidar.vertical_fov",
                                 lidar_params.vertical_fov, 64.0);

  this->get_parameter_or<vector<double>>("lidar.t_imu_lidar", t_imu_lidar,
                                         vector<double>());
  this->get_parameter_or<vector<double>>("lidar.r_imu_lidar", r_imu_lidar,
                                         vector<double>());

  this->get_parameter_or<int>("cameras.num_cams", num_cams, 0);
  this->get_parameter_or<string>("cameras.transport", cam_transport, "raw");
  this->get_parameter_or<vector<long int>>("cameras.frame_rates",
                                           cam_frame_rates, vector<long int>());
  this->get_parameter_or<vector<string>>("cameras.cam_topics", cam_topics,
                                         vector<string>());
  this->get_parameter_or<vector<double>>("cameras.cam_intrinsics",
                                         cam_intrinsics, vector<double>());
  this->get_parameter_or<vector<double>>("cameras.t_cam_lidars", t_cam_lidars,
                                         vector<double>());
  this->get_parameter_or<vector<double>>("cameras.r_cam_lidars", r_cam_lidars,
                                         vector<double>());

  map_cloud->reserve(MAX_MAP_POINTS);
  raw_cloud->reserve(MAX_PROC_POINTS);
  scan_cloud->reserve(MAX_PROC_POINTS);
  filter_cloud->reserve(MAX_PROC_POINTS);
  buffer_cloud->reserve(MAX_PROC_POINTS);

  map_resolution = fmax(map_resolution, MIN_MAP_RES);
  map_search_rad = 10 * map_resolution;

  ioctree.set_bucket_size(1);
  ioctree.set_max_octants(MAX_MAP_POINTS);
  ioctree.set_max_new_points(MAX_PROC_POINTS);
  ioctree.set_min_extent(map_resolution);

  ekfom_data_w = Eigen::ArrayXd(MAX_PROC_POINTS);
  ekfom_data_om = Eigen::ArrayXd::Zero(100);
  ekfom_data_ot = Eigen::ArrayXXd(MAX_PROC_POINTS, 3);
  ekfom_data_or = Eigen::ArrayXXd(MAX_PROC_POINTS, 3);
  ekfom_data_h = Eigen::VectorXd(MAX_PROC_POINTS);
  ekfom_data_w_x = Eigen::ArrayXd(MAX_PROC_POINTS);
  ekfom_data_h_x = Eigen::MatrixXd(MAX_PROC_POINTS, 6);
  ekfom_data_h_x_R = Eigen::MatrixXd(6, MAX_PROC_POINTS);

  ekfom_data_h_v = Eigen::ArrayXd(MAX_PROC_POINTS);
  ekfom_data_h_x_v = Eigen::MatrixXd(MAX_PROC_POINTS, 6);
  ekfom_data_oit = Eigen::ArrayXXd(MAX_PROC_POINTS, 3);
  ekfom_data_oir = Eigen::ArrayXXd(MAX_PROC_POINTS, 3);
  // if (num_cams) {
  //   sh_mats = std::vector<Eigen::MatrixXf>(
  //       MAX_MAP_POINTS, Eigen::MatrixXf(3, harmonics->getNumCoeffs()));
  // }

  colors.reserve(MAX_MAP_POINTS);
  valid_reg.reserve(MAX_MAP_POINTS);
  update_idx.reserve(MAX_MAP_POINTS);
  saliency_idxs.reserve(MAX_MAP_POINTS);
  neighbours.reserve(MAX_MAP_POINTS);
  filters.reserve(MAX_MAP_POINTS);

  tensors_p1.reserve(MAX_MAP_POINTS);
  tensors_p2.reserve(MAX_MAP_POINTS);
  salivalues.reserve(MAX_MAP_POINTS);
  eigenvalues.reserve(MAX_MAP_POINTS);
  eigenvectors.reserve(MAX_MAP_POINTS);

  updated_pt = std::vector<std::atomic<int>>(MAX_MAP_POINTS);
  new_neighbours_map_idx = std::vector<int>(MAX_SCAN_POINTS);
  new_neighbours_size = std::vector<std::atomic<int>>(MAX_SCAN_POINTS);
  new_neighbours = std::vector<std::vector<int>>(
      MAX_SCAN_POINTS, std::vector<int>(MAX_NEIGHBOURS));

  analytics_msg_ = ellipse_lio::msg::EllipseLioAnalytics();
  analytics_msg_pub_ = ellipse_lio::msg::EllipseLioAnalytics();

  last_imu_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  imu_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  imu_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  raw_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  raw_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  scan_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  scan_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  buffer_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  buffer_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  imu_params.t_imu_lidar << VEC_FROM_ARRAY(t_imu_lidar);
  if (r_imu_lidar.size() == 9) {
    imu_params.r_imu_lidar << MAT_FROM_ARRAY(r_imu_lidar);
  } else if (r_imu_lidar.size() == 4) {
    imu_params.r_imu_lidar =
        Eigen::Quaterniond(QUAT_FROM_ARRAY(r_imu_lidar)).toRotationMatrix();
  } else {
    RCLCPP_ERROR(
        this->get_logger(),
        "Lidar to IMU rotation is not a valid quaternion or rotation matrix");
  }

  double epsi[23];
  std::fill_n(epsi, 23, 0.001);
  kf_->init_dyn_share(get_f, df_dx, df_dw,
                      std::bind(&MappingNode::tensor_registration, this,
                                std::placeholders::_1, std::placeholders::_2),
                      10, epsi);

  last_imu_pub_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_opt_pub_time = rclcpp::Time(0, 0, RCL_ROS_TIME);

  loop_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pub_map_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pub_odo_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  pub_analytics_ =
      this->create_publisher<ellipse_lio::msg::EllipseLioAnalytics>(
          "/analytics", rclcpp::SensorDataQoS());
  pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud_map", rclcpp::SensorDataQoS());
  pub_scan_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud_scan", rclcpp::SensorDataQoS());
  pub_mark_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/visualization_marker", rclcpp::SensorDataQoS());

  loop_timer_ = rclcpp::create_timer(
      this, this->get_clock(),
      std::chrono::milliseconds(1000 / imu_params.rate),
      std::bind(&MappingNode::timer_callback, this), loop_callback_group_);
  pub_odo_timer_ =
      rclcpp::create_timer(this, this->get_clock(),
                           std::chrono::milliseconds(1000 / lidar_params.rate),
                           std::bind(&MappingNode::publish_opt_odometry, this),
                           pub_odo_callback_group_);
  pub_map_timer_ = rclcpp::create_timer(
      this, this->get_clock(), std::chrono::milliseconds(pub_map_n_secs * 1000),
      std::bind(&MappingNode::publish_map, this), pub_map_callback_group_);

  char line[128];
  struct tms timeSample;
  lastCPU = times(&timeSample);
  lastSysCPU = timeSample.tms_stime;
  lastUserCPU = timeSample.tms_utime;

  FILE* file;
  file = fopen("/proc/cpuinfo", "r");
  numProcessors = 0;
  while (fgets(line, 128, file) != nullptr) {
    if (strncmp(line, "processor", 9) == 0) numProcessors++;
  }
  fclose(file);

  start_time = omp_get_wtime();
  RCLCPP_INFO(this->get_logger(), "Node init finished.");
}

MappingNode::~MappingNode() {}

// Initialize camera processes
void MappingNode::init_cam_process() {
  if (num_cams == 0) return;

  if (cam_frame_rates.size() != num_cams) {
    RCLCPP_ERROR(this->get_logger(), "Frame rates and num cameras mismatch");
    exit(1);
  }
  if (cam_topics.size() != num_cams) {
    RCLCPP_ERROR(this->get_logger(), "Cam topics and num cameras mismatch");
    exit(1);
  }
  if (t_cam_lidars.size() != num_cams * 3) {
    RCLCPP_ERROR(this->get_logger(),
                 "Cam translations and num cameras mismatch");
    exit(1);
  }
  if (!(r_cam_lidars.size() == num_cams * 9 ||
        r_cam_lidars.size() == num_cams * 4)) {
    RCLCPP_ERROR(this->get_logger(), "Cam rotations and num cameras mismatch");
    exit(1);
  }
  if (cam_intrinsics.size() != num_cams * 9) {
    RCLCPP_ERROR(this->get_logger(), "Cam intrinsics and num cameras mismatch");
    exit(1);
  }

  for (int i = 0; i < num_cams; i++) {
    CamParams cam_params;

    cam_params.topic = cam_topics[i];
    cam_params.rate = cam_frame_rates[i];
    cam_params.transport = cam_transport;

    vector<double> t_cam_lidar(t_cam_lidars.begin() + i * 3,
                               t_cam_lidars.begin() + i * 3 + 3);
    vector<double> cam_intrinsic(cam_intrinsics.begin() + i * 9,
                                 cam_intrinsics.begin() + i * 9 + 9);

    cam_params.t_cam_lidar << VEC_FROM_ARRAY(t_cam_lidar);
    cam_params.cam_intrinsics << MAT_FROM_ARRAY(cam_intrinsic);

    if (r_cam_lidars.size() == num_cams * 4) {
      vector<double> r_cam_lidar(r_cam_lidars.begin() + i * 4,
                                 r_cam_lidars.begin() + i * 4 + 4);
      cam_params.r_cam_lidar =
          Eigen::Quaterniond(QUAT_FROM_ARRAY(r_cam_lidar)).toRotationMatrix();
    } else {
      vector<double> r_cam_lidar(r_cam_lidars.begin() + i * 9,
                                 r_cam_lidars.begin() + i * 9 + 9);
      cam_params.r_cam_lidar << MAT_FROM_ARRAY(r_cam_lidar);
    }

    cams_process.push_back(
        std::make_shared<CamProcess>(cam_params, shared_from_this()));
  }
}

void MappingNode::sync_raw_cloud_with_imu() {
  int total_size;
  std::atomic<int> scan_idx = 0, filter_idx = 0;

  imu_time_offset_ = 0;
  imu_process->SyncWithLidar(imu_start_time_, imu_end_time_);
  lid_time_offset_ = lid_process->lidar_time_offset_.seconds();

  if (!raw_cloud->empty()) {
    scan_start_time_ = raw_start_time_;
    scan_end_time_ = raw_end_time_;
  }
  if (!buffer_cloud->empty()) scan_start_time_ = buffer_start_time_;
  if (imu_end_time_ > scan_end_time_ && imu_start_time_ < scan_start_time_) {
    *scan_cloud = *buffer_cloud;
    *scan_cloud += *raw_cloud;
    scan_cloud_bins = buffer_cloud_bins + raw_cloud_bins;
    buffer_cloud->clear();
    buffer_cloud_bins.setZero();
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

    std::fill(scan_bin_sizes.begin(), scan_bin_sizes.end(), 0);
    std::fill(filter_bin_sizes.begin(), filter_bin_sizes.end(), 0);
    total_size = buffer_cloud->size() + raw_cloud->size();

    scan_cloud->resize(total_size);
    filter_cloud->resize(total_size);

#pragma omp parallel for
    for (int i = 0; i < total_size; i++) {
      const EllipseLioPoint& pt =
          i < buffer_cloud->size()
              ? buffer_cloud->points[i]
              : raw_cloud->points[i - buffer_cloud->size()];
      rclcpp::Time pt_time =
          rclcpp::Time(pt.time_secs, pt.time_nsecs, RCL_ROS_TIME);
      if (pt_time < scan_start_time_) continue;
      if (pt_time > scan_end_time_) {
        filter_bin_sizes[pt.bin_idx]++;
        filter_cloud->points[filter_idx++] = pt;
      } else {
        scan_bin_sizes[pt.bin_idx]++;
        scan_cloud->points[scan_idx++] = pt;
      }
    }

    scan_cloud->resize(scan_idx);
    filter_cloud->resize(filter_idx);
    *buffer_cloud = *filter_cloud;

#pragma omp parallel for
    for (int i = 0; i < scan_bin_sizes.size(); i++) {
      scan_cloud_bins[i] = scan_bin_sizes[i];
      buffer_cloud_bins[i] = filter_bin_sizes[i];
    }
  }

  raw_cloud->clear();
  raw_cloud_bins.setZero();
  raw_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  raw_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  analytics_msg_.imu_offset = imu_time_offset_;
  analytics_msg_.lid_offset = lid_time_offset_;
  analytics_msg_.scan_size = scan_cloud->size();
  analytics_msg_.buffer_size = buffer_cloud->size();
  analytics_msg_.scan_time = (scan_end_time_ - scan_start_time_).seconds();
}

void MappingNode::compute_ram_usage() {
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

void MappingNode::compute_cpu_usage() {
  struct tms timeSample;
  clock_t now;
  double cpu_percent;
  now = times(&timeSample);
  if (now <= lastCPU || timeSample.tms_stime < lastSysCPU ||
      timeSample.tms_utime < lastUserCPU) {
    cpu_percent = -1.0;
  } else {
    cpu_percent = (timeSample.tms_stime - lastSysCPU) +
                  (timeSample.tms_utime - lastUserCPU);
    cpu_percent /= (now - lastCPU);
    cpu_percent /= numProcessors;
    cpu_percent *= 100.;
  }
  lastCPU = now;
  lastSysCPU = timeSample.tms_stime;
  lastUserCPU = timeSample.tms_utime;

  analytics_msg_.cpu_usage = round(cpu_percent);
}

// Main mapping loop
void MappingNode::timer_callback() {
  if (!initialized) {
    initialized = true;
    imu_process =
        std::make_shared<ImuProcess>(kf_, imu_params, shared_from_this());
    lid_process = std::make_shared<LidarProcess>(lidar_params, map_resolution,
                                                 shared_from_this());
    init_cam_process();

    n_res = Eigen::ArrayXf::Ones(lidar_params.rate);
    n_res *= 0.5;
    n_means = Eigen::ArrayXi::Zero(lid_process->num_bins_);
    n_cnts = Eigen::ArrayXXi::Zero(MAX_PROC_POINTS, lid_process->num_bins_);
    n_bins = Eigen::ArrayXXi::Zero(MAX_PROC_POINTS, lid_process->num_bins_);

    raw_cloud_bins = Eigen::ArrayXi::Zero(lid_process->num_bins_);
    scan_cloud_bins = Eigen::ArrayXi::Zero(lid_process->num_bins_);
    buffer_cloud_bins = Eigen::ArrayXi::Zero(lid_process->num_bins_);
    scan_bin_sizes = std::vector<std::atomic<int>>(lid_process->num_bins_);
    filter_bin_sizes = std::vector<std::atomic<int>>(lid_process->num_bins_);

    sep_factor = std::vector<int>(lid_process->num_bins_, 10);
    init_poses = std::vector<bool>(lid_process->num_bins_, false);
    last_updated_poses = std::vector<Eigen::Vector3f>(lid_process->num_bins_);
    last_updated_rotes =
        std::vector<Eigen::Quaternionf>(lid_process->num_bins_);
  }

  if (sync_packages()) {
    double t1, t2, t3, t4, imu_time, state_time, map_time, total_time;

    t1 = omp_get_wtime();

    imu_process->UndistortPointCloud(scan_cloud, kf_state_, scan_start_time_,
                                     scan_end_time_, cams_process);

    t2 = omp_get_wtime();

    if (map_counter) {
      ekfom_div_cnt = 0;
      ekfom_iter_cnt = 0;
      imu_process->UpdateStatesWithLidar(kf_state_, scan_end_time_,
                                         0.5 / lidar_params.rate);
    }

    t3 = omp_get_wtime();

    map_mutex_.lock();
    map_incremental();
    map_mutex_.unlock();

    t4 = omp_get_wtime();

    imu_time = t2 - t1;
    state_time = t3 - t2;
    map_time = t4 - t3;
    total_time = t4 - t1;

    max_imu_time = fmax(max_imu_time, imu_time);
    max_state_time = fmax(max_state_time, state_time);
    max_map_time = fmax(max_map_time, map_time);
    max_total_time = fmax(max_total_time, total_time);

    mean_imu_time += imu_time;
    mean_state_time += state_time;
    mean_map_time += map_time;
    mean_total_time += total_time;

    compute_ram_usage();
    compute_cpu_usage();

    analytics_msg_.run_time = round(t4 - start_time);

    analytics_msg_.imu_time = imu_time;
    analytics_msg_.state_time = state_time;
    analytics_msg_.map_time = map_time;
    analytics_msg_.total_time = total_time;

    analytics_msg_.imu_mean = mean_imu_time / (map_counter + 1);
    analytics_msg_.state_mean = mean_state_time / (map_counter + 1);
    analytics_msg_.map_mean = mean_map_time / (map_counter + 1);
    analytics_msg_.total_mean = mean_total_time / (map_counter + 1);

    analytics_msg_.imu_max = max_imu_time;
    analytics_msg_.state_max = max_state_time;
    analytics_msg_.map_max = max_map_time;
    analytics_msg_.total_max = max_total_time;

    odom_mutex_.lock();
    kf_state_pub_ = kf_state_;
    *scan_cloud_pub = *scan_cloud;
    analytics_msg_pub_ = analytics_msg_;
    odom_mutex_.unlock();

    map_counter++;
  }
}

}  // namespace ellipselio

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ellipselio::MappingNode)