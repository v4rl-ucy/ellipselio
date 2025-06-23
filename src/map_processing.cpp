#include <map_processing.h>

namespace ellipselio {

// Sync lidar, imu, and camera data
bool MappingNode::sync_packages() {
  double inter_sync_time = omp_get_wtime() - last_sync_time;

  if (!last_sync_time) {
    if (int(ceil(inter_sync_time / 0.01)) % 100 == 0) {
      RCLCPP_INFO(this->get_logger(), "Waiting for data...");
    }
  }
  if (!imu_process->imu_has_data_) {
    if (int(ceil(inter_sync_time / 0.01)) % 20 == 0) {
      RCLCPP_ERROR(this->get_logger(), "IMU has no data");
    }
    return false;
  }
  if (!lid_process->lidar_has_data_) {
    if (int(ceil(inter_sync_time / 0.01)) % 20 == 0) {
      RCLCPP_ERROR(this->get_logger(), "Lidar has no data");
    }
    return false;
  }
  if (imu_process->imu_end_time_ < lid_process->lidar_end_time_) {
    if (int(ceil(inter_sync_time / 0.01)) % 20 == 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "IMU end time is less than lidar end time");
      RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "IMU end time: " << imu_process->imu_end_time_.nanoseconds());
      RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "Lidar end time: " << lid_process->lidar_end_time_.nanoseconds());
    }
    return false;
  }
  if (imu_process->imu_start_time_ > lid_process->lidar_start_time_) {
    if (int(ceil(inter_sync_time / 0.01)) % 20 == 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "IMU start time is greater than lidar start time");
      RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "IMU start time: " << imu_process->imu_start_time_.nanoseconds());
      RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "Lidar start time: " << lid_process->lidar_start_time_.nanoseconds());
    }
    lid_process->ClearPointCloud();
    return false;
  }
  for (int i = 0; i < num_cams; i++) {
    if (!cams_process[i]->cam_has_data_) {
      if (int(ceil(inter_sync_time / 0.01)) % 20 == 0) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Camera " << i << " has no data");
      }
      return false;
    }
  }
  for (int i = 0; i < num_cams; i++) {
    if (cams_process[i]->img_end_time_ < imu_process->imu_start_time_) {
      if (int(ceil(inter_sync_time / 0.01)) % 20 == 0) {
        RCLCPP_ERROR_STREAM(
            this->get_logger(),
            "Camera " << i << " end time is less than imu start time");
      }
      return false;
    }
  }

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
void MappingNode::compute_tensor_vote(int i, int j, M3F &A_j, bool first_pass) {
  V3F p_i = map_cloud->points[i].getVector3fMap();
  V3F p_j = map_cloud->points[j].getVector3fMap();
  const int &bin_idx = map_cloud->points[i].bin_idx;
  const float &search_rad = lid_process->search_radii_[bin_idx];
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
void MappingNode::compute_tensor_eigen(int i, M3F &tensor, bool first_pass) {
  V3F eig_val, sali_val;
  M3F eig_vec, tensor_i2;
  Eigen::SelfAdjointEigenSolver<M3F> eig_solver;

  eig_solver.computeDirect(tensor);
  eig_vec = eig_solver.eigenvectors();
  eig_val = eig_solver.eigenvalues().cwiseAbs();

  const int &bin_idx = map_cloud->points[i].bin_idx;

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
    eigenvalues[i] *= lid_process->search_radii_[bin_idx];
    eigenvectors[i] = eig_vec;
    map_cloud->points[i].intensity = (saliency_idxs[i] + 1) * 85;
  }
}

// Compute first pass tensor voting for new points and find neighbours
void MappingNode::tensor_vote_pass_1(int old_map_size,
                                     std::vector<int> &added_idxs,
                                     std::vector<int> &updated_idxs) {
  std::atomic<int> upd_idx = 0, new_neighbours_idx = 0;
  int num_bins = lid_process->num_bins_;
  Eigen::ArrayXi n_cnt = Eigen::ArrayXi::Zero(added_idxs.size());
  Eigen::ArrayXXi n_bins = Eigen::ArrayXXi::Zero(added_idxs.size(), num_bins);

#pragma omp parallel for
  for (int i = 0; i < added_idxs.size(); i++) {
    int map_i;
    std::vector<int> N_idxs;

    map_i = added_idxs[i];
    valid_reg[map_i] = 1;
    count_reg[map_i] = 1;
    updated_pt[map_i] = 0;
    map_cloud->points[map_i].intensity = 0;
    colors[map_i] =
        map_cloud->points[map_i].getRGBVector3i().cast<float>() / 255.0f;

    const int &bin_idx = map_cloud->points[map_i].bin_idx;
    const int &bucket_size = lid_process->bucket_sizes_[bin_idx];
    const float &search_rad = lid_process->search_radii_[bin_idx];

    ioctree.radiusNeighbors(map_cloud->points[map_i], search_rad, N_idxs,
                            bucket_size);
    neighbours[map_i] = N_idxs;
    n_cnt(i) = N_idxs.size();
    n_bins(i, bin_idx) = 1;
  }

#pragma omp parallel for
  for (int i = 0; i < num_bins; i++) {
    Eigen::ArrayXi n_cnt_bin = n_cnt * n_bins.col(i);
    if (!n_cnt_bin.sum()) continue;
    int n_mean = lid_process->min_neighbours_[i];
    n_mean *= lid_process->cnt_neighbours_[i];
    n_mean += n_cnt_bin.sum();
    n_mean /= lid_process->cnt_neighbours_[i] + n_bins.col(i).sum();
    lid_process->min_neighbours_[i] =
        fmin(fmax(n_mean, MIN_NEIGHBOURS), MAX_NEIGHBOURS);
    lid_process->max_neighbours_[i] =
        fmin(2 * lid_process->min_neighbours_[i], MAX_NEIGHBOURS);
    lid_process->cnt_neighbours_[i] += n_bins.col(i).sum();
  }

#pragma omp parallel for
  for (int i = 0; i < added_idxs.size(); i++) {
    int map_i, loop_cnt;
    Eigen::MatrixXf K;
    M3F tensor_i1;

    map_i = added_idxs[i];

    const int &bin_idx = map_cloud->points[map_i].bin_idx;
    const int &min_neigh = lid_process->min_neighbours_[bin_idx];
    const int &max_neigh = lid_process->max_neighbours_[bin_idx];

    loop_cnt = min(int(neighbours[map_i].size()), max_neigh);
    K = Eigen::MatrixXf::Zero(loop_cnt, 9);

#pragma omp parallel for
    for (int j = 0; j < loop_cnt; j++) {
      M3F A_j;
      int map_j = neighbours[map_i][j];
      compute_tensor_vote(map_i, map_j, A_j, true);
      K.row(j) = A_j.reshaped(1, 9);

      if (map_j >= old_map_size) continue;

      const int &bin_idx_j = map_cloud->points[map_j].bin_idx;
      const float &search_rad_j = lid_process->search_radii_[bin_idx_j];
      EllipseLioPoint &pt_i = map_cloud->points[map_i];
      EllipseLioPoint &pt_j = map_cloud->points[map_j];
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

    const int &bin_idx = map_cloud->points[map_i].bin_idx;
    const int &min_neigh = lid_process->min_neighbours_[bin_idx];
    const int &max_neigh = lid_process->max_neighbours_[bin_idx];
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
void MappingNode::tensor_vote_pass_2(std::vector<int> &added_idxs,
                                     std::vector<int> &updated_idxs) {
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

    const int &bin_idx = map_cloud->points[map_i].bin_idx;
    const int &min_neigh = lid_process->min_neighbours_[bin_idx];
    const int &max_neigh = lid_process->max_neighbours_[bin_idx];
    loop_cnt = min(int(neighbours[map_i].size()), max_neigh);

    if (!filters[map_i][0]) continue;

    K = Eigen::MatrixXf::Zero(loop_cnt, 9);
    K_filter = Eigen::VectorXi::Zero(loop_cnt);

    if (num_cams) {
      SH_filter = Eigen::VectorXi::Zero(loop_cnt + 1);
      SH = SHCoeffs(loop_cnt + 1, harmonics->getNumCoeffs());
    }
    if (map_cloud->points[map_i].has_rgb) {
      SH_filter(loop_cnt) = 1;
      compute_harmonics(map_i, map_i, loop_cnt, SH);
    }

#pragma omp parallel for
    for (int j = 0; j < loop_cnt; j++) {
      int map_j = neighbours[map_i][j];
      if (!filters[map_j][0]) continue;

      M3F A_j;
      compute_tensor_vote(map_i, map_j, A_j, false);
      K.row(j) = A_j.reshaped(1, 9);
      K_filter(j) = 1;

      if (map_cloud->points[map_j].has_rgb) {
        SH_filter(j) = 1;
        compute_harmonics(map_i, map_j, j, SH);
      }
    }

    filter_cnt = K_filter.sum();
    if (filter_cnt < min_neigh) continue;

    tensor_i2 = K.colwise().sum().reshaped(3, 3);
    tensor_i2 /= float(filter_cnt);
    compute_tensor_eigen(map_i, tensor_i2, false);

    if (num_cams) {
      color_cnt = SH_filter.sum();
      if (color_cnt < min_neigh) continue;

      sh_dir = poses[map_cloud->points[map_i].scan_idx];
      sh_dir -= map_cloud->points[map_i].getVector3fMap();
      harmonics->finalizeCoefficients(SH, sh_mats[map_i]);
      harmonics->evaluateColorFromDirection(sh_mats[map_i], sh_dir, sh_color);
      map_cloud->points[map_i].r = sh_color(0) * 255.0f;
      map_cloud->points[map_i].g = sh_color(1) * 255.0f;
      map_cloud->points[map_i].b = sh_color(2) * 255.0f;
    }
  }
}

void MappingNode::compute_harmonics(int map_i, int map_j, int loop_idx,
                                    SHCoeffs &SH) {
  Eigen::Vector3f dir;

  const int &bin_idx = map_cloud->points[map_i].bin_idx;
  const float &search_rad = bin_idx;
  const Eigen::Vector3f &pose = poses[map_cloud->points[map_j].scan_idx];
  const Eigen::Vector3f &p_i = map_cloud->points[map_i].getVector3fMap();
  const Eigen::Vector3f &p_j = map_cloud->points[map_j].getVector3fMap();

  harmonics->dirFromNeighbouringPoint(p_i, p_j, pose, dir, search_rad);
  harmonics->computeCoefficients(dir, colors[map_j], SH, loop_idx);
}

// Add new points to the map and update geometric primitives
void MappingNode::map_incremental() {
  int start_idx, end_idx;
  std::vector<int> new_idxs, updated_idxs, added_idxs_i, new_idxs_i;

  poses[map_counter] = kf_state_.state.pos.cast<float>();

#pragma omp parallel for
  for (int i = 0; i < scan_cloud->size(); i++) {
    scan_cloud->points[i].scan_idx = map_counter;
    const int &bin_idx = scan_cloud->points[i].bin_idx;
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

    ioctree.set_bucket_size(lid_process->bucket_sizes_[fmax(i, start_bin)]);
    ioctree.update(*scan_cloud, added_idxs_i, new_idxs_i, true, start_idx,
                   end_idx);
    *map_cloud += EllipseLioPointCloud(*scan_cloud, added_idxs_i);
    new_idxs.insert(new_idxs.end(), new_idxs_i.begin(), new_idxs_i.end());
    start_idx = end_idx;
  }
  new_map_size = map_cloud->size();

  last_reg.resize(map_cloud->size(), 0);
  valid_reg.resize(map_cloud->size(), 0);
  update_idx.resize(map_cloud->size(), 0);
  saliency_idxs.resize(map_cloud->size(), 0);
  poses.resize(map_cloud->size(), V3F::Zero());
  colors.resize(map_cloud->size(), Eigen::Vector3f::Zero());
  neighbours.resize(map_cloud->size(), std::vector<int>());
  filters.resize(map_cloud->size(), std::vector<bool>(2, false));

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

void MappingNode::split_map(const sensor_msgs::msg::PointCloud2 &input,
                            std::vector<sensor_msgs::msg::PointCloud2> &clouds,
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
  for (auto &part : map_parts) {
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
    marker.ns = "map_primitives";
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
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = 2 * eigenvalues[map_idx](0);
        marker.scale.y = 2 * eigenvalues[map_idx](1);
        marker.scale.z = 2 * eigenvalues[map_idx](2);
        marker.color.r = 1.0;
        break;
      case 1:
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = 2 * eigenvalues[map_idx](0);
        marker.scale.y = 2 * eigenvalues[map_idx](1);
        marker.scale.z = 2 * eigenvalues[map_idx](2);
        marker.color.g = 1.0;
        break;
      case 2:
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = 2 * eigenvalues[map_idx](0);
        marker.scale.y = 2 * eigenvalues[map_idx](1);
        marker.scale.z = 2 * eigenvalues[map_idx](2);
        marker.color.b = 1.0;
        break;
    }
    marker_array.markers[marker_idx++] = marker;
  }
  last_map_size = new_map_size;
  marker_array.markers.resize(marker_idx);
  pub_mark_->publish(marker_array);
}

// Publish odometry transform
void MappingNode::publish_odometry() {
  if (!map_counter) return;

  if (last_pub_time == kf_state_pub_.time) return;
  odom_mutex_.lock();
  last_pub_time = kf_state_pub_.time;

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
    state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
  double t0, t1, res_mean;
  float wt_min, wt_max, wt_mean, wt_std;
  int feat_tot, plane_tot, line_tot, pt_tot, reject_cnt;
  float rng_min, rng_max, rng_mean, rng_min_scale, rng_max_scale;

  V3F hit_mean;
  Eigen::Array3i feats_num(3), cnts(3);
  std::vector<std::atomic<int>> prim_cnts(3);
  Eigen::ArrayXd means(9), maxs(9), mins(9), stds(9);

  prim_cnts[0] = 0;
  prim_cnts[1] = 0;
  prim_cnts[2] = 0;

  maxs.setZero();
  mins.setZero();
  stds.setZero();
  cnts.setZero();
  means.setZero();
  hit_mean.setZero();
  feats_num.setZero();

  if (ekfom_iter_cnt > 0) {
    if (max_ekfom_time - ekfom_iter_time < ekfom_iter_time / ekfom_iter_cnt) {
      ekfom_data.valid = false;
      return;
    }
  }

  t0 = omp_get_wtime();
  const int &max_start_bin = lid_process->max_start_bin_;

#pragma omp parallel for
  for (int i = 0; i < scan_cloud->size(); i++) {
    Eigen::VectorXd h_x_vec(6);
    std::vector<int> N_idxs, N_p_idxs;
    std::vector<float> N_dst, N_p_dst;
    rclcpp::Time map_pt_time, scan_pt_time;
    int sali_idx, map_i, feat_num, prim_num;
    float residual, prim_score, time_score, ellipse_score, total_score;

    M3D P_skew;
    V3D p_lidar, p_imu, a;
    V3F sali_vals, scores, p_world, n_world, p_dash, q, q_dash, norm_vec;

    const EllipseLioPoint &pt = scan_cloud->points[i];

    p_lidar = pt.getVector3fMap().cast<double>();
    p_imu = s.offset_R_L_I * p_lidar + s.offset_T_L_I;
    p_world = (s.rot * p_imu + s.pos).cast<float>();

    const int scan_bin_idx = fmax(scan_cloud->points[i].bin_idx, start_bin);
    const float &search_rad = lid_process->search_radii_[scan_bin_idx];

    ioctree.knnNeighbors(p_world, 1, N_idxs, N_dst, search_rad);
    if (N_idxs.size() == 0) continue;
    if (!filters[N_idxs[0]][1]) continue;

    map_i = N_idxs[0];
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

    q_dash = eigenvectors[map_i].transpose() * (p_dash - n_world);

    prim_score = 1 - scores.maxCoeff();
    time_score = 1.0 / ((scan_pt_time - map_pt_time).seconds() + 1.1);
    time_score = pow(time_score, (start_bin + 1.0) / (max_start_bin + 1.0));
    ellipse_score = q_dash.cwiseQuotient(eigenvalues[map_i]).cwiseAbs2().sum();

    prim_score = fmin(fmax(prim_score, 1e-3), 1.0);
    time_score = fmin(fmax(time_score, 1e-3), 1.0);
    ellipse_score = fmin(fmax(ellipse_score, 1e-3), 1.0);

    scores(0) = time_score;
    scores(1) = prim_score;
    scores(2) = ellipse_score;
    scores(1) *= 1.0 / round(1.0 / fmin(scores(0) / (10 * scores(1)), 1));
    scores(2) *= 1.0 / round(1.0 / fmin(scores(0) / (10 * scores(2)), 1));
    total_score = 1.0 / fmin(scores.sum(), 1.0);

    P_skew << SKEW_SYM_MATRIX(p_imu);
    a = P_skew * s.rot.conjugate() * norm_vec.cast<double>();
    h_x_vec << norm_vec(0), norm_vec(1), norm_vec(2), a[0], a[1], a[2];

    prim_num = ++prim_cnts[saliency_idxs[map_i]];
    ekfom_data_i(prim_num - 1, saliency_idxs[map_i]) = map_i;
    ekfom_data_c(prim_num - 1, saliency_idxs[map_i]) = count_reg[map_i];
    ekfom_data_w(prim_num - 1, saliency_idxs[map_i]) = total_score;
    ekfom_data_w(prim_num - 1, saliency_idxs[map_i] + 3) = prim_score;
    ekfom_data_w(prim_num - 1, saliency_idxs[map_i] + 6) = ellipse_score;

    ekfom_data_h_v[saliency_idxs[map_i]](prim_num - 1) = -residual;
    ekfom_data_h_x_v[saliency_idxs[map_i]].row(prim_num - 1) = h_x_vec;
  }

  cnts << prim_cnts[0].load(), prim_cnts[1].load(), prim_cnts[2].load();
  feat_tot = cnts.sum();

#pragma omp parallel for
  for (int i = 0; i < 9; i++) {
    if (!cnts(i % 3)) continue;
    means(i) = ekfom_data_w.col(i).head(cnts(i % 3)).mean();
    mins(i) = ekfom_data_w.col(i).head(cnts(i % 3)).minCoeff();
    maxs(i) = ekfom_data_w.col(i).head(cnts(i % 3)).maxCoeff();
    stds(i) = (ekfom_data_w.col(i).head(cnts(i % 3)) - means(i)).square().sum();
    stds(i) = sqrt(stds(i) / (cnts(i % 3) - 1));
  }

  wt_min = mins.head(3).minCoeff();
  wt_max = maxs.head(3).maxCoeff();
  wt_std = stds.head(3).maxCoeff();
  wt_mean = means.head(3).maxCoeff();
  rng_min = wt_mean - wt_std;
  rng_min = fmax(rng_min, wt_min);
  rng_min_scale = rng_min / wt_min;
  rng_mean = wt_mean - rng_min;
  rng_max = wt_max - rng_min;
  rng_max = fmin(rng_mean + wt_std, rng_max);
  rng_max_scale = rng_max / ((wt_max * rng_min_scale) - rng_min);

#pragma omp parallel for
  for (int i = 0; i < 3; i++) {
    int st, sz;
    float std_p, std_e, hit_bin;

    if (!cnts(i)) continue;

    if (stds(i)) {
      ekfom_data_w.col(i).head(cnts(i)) *= rng_min_scale;
      ekfom_data_w.col(i).head(cnts(i)) -= rng_min;
      ekfom_data_w.col(i).head(cnts(i)) *= rng_max_scale;
      ekfom_data_w.col(i).head(cnts(i)) += 1.0;
    }

    if (stds(i + 3) && stds(i + 6)) {
      hit_mean(i) = ekfom_data_c.col(i).head(cnts(i)).mean();

      hit_bin = fmax(start_bin / 3.0, 1.0);
      std_p = stds(i + 3) * pow(hit_mean(i), 1.0 / hit_bin);
      std_e = stds(i + 6) * pow(hit_mean(i), 1.0 / hit_bin);

      ekfom_data_v.col(i).head(cnts(i)) =
          (ekfom_data_w.col(i + 3).head(cnts(i)) < means(i + 3) + std_p &&
           ekfom_data_w.col(i + 6).head(cnts(i)) < means(i + 6) + std_e)
              .cast<double>();
      feats_num(i) = ekfom_data_v.col(i).head(cnts(i)).sum();
      ekfom_data_w.col(i).head(cnts(i)) *= ekfom_data_v.col(i).head(cnts(i));
      ekfom_data_h_v[i].head(cnts(i)) *= ekfom_data_v.col(i).head(cnts(i));

#pragma omp parallel for
      for (int j = 0; j < cnts(i); j++) {
        if (last_reg[ekfom_data_i(j, i)] < map_counter) {
          last_reg[ekfom_data_i(j, i)] = map_counter;
          count_reg[ekfom_data_i(j, i)]++;
        }
        valid_reg[ekfom_data_i(j, i)] = ekfom_data_v(j, i);
      }
    }

    if (i == 0) {
      st = 0;
    } else {
      st = cnts.head(i).sum();
    }
    sz = cnts(i);

    ekfom_data_h.block(st, 0, sz, 1) = ekfom_data_h_v[i].head(cnts(i));
    ekfom_data_w_x.block(st, 0, sz, 1) = ekfom_data_w.col(i).head(cnts(i));
    ekfom_data_h_x.block(st, 0, sz, 6) = ekfom_data_h_x_v[i].topRows(cnts(i));
  }
  rng_min = 1;
  rng_max += 1;

  ekfom_data_h_x_R.leftCols(feat_tot) =
      (ekfom_data_h_x.topRows(feat_tot).array().colwise() *
       ekfom_data_w_x.head(feat_tot))
          .transpose();

  ekfom_data.h = ekfom_data_h.head(feat_tot);
  ekfom_data.h_x = ekfom_data_h_x.topRows(feat_tot);
  ekfom_data.h_x_R = ekfom_data_h_x_R.leftCols(feat_tot);

  res_mean = -ekfom_data_h.head(feat_tot).sum();

  feat_tot = feats_num.sum();
  res_mean /= feat_tot;
  reject_cnt = scan_cloud->size() - feat_tot;

  analytics_msg_.wt_std = wt_std;
  analytics_msg_.wt_min = wt_min;
  analytics_msg_.wt_max = wt_max;
  analytics_msg_.wt_mean = wt_mean;
  analytics_msg_.rng_min = rng_min;
  analytics_msg_.rng_max = rng_max;
  analytics_msg_.rng_mean = rng_mean;
  analytics_msg_.res_mean = res_mean;
  analytics_msg_.start_bin = start_bin;
  analytics_msg_.num_feats = feat_tot;
  analytics_msg_.num_reject = reject_cnt;
  analytics_msg_.hit_mean = hit_mean.mean();

  ekfom_iter_cnt++;
  t1 = omp_get_wtime();
  ekfom_iter_time += t1 - t0;
}

// Main mapping node
MappingNode::MappingNode(
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node("mapping_node", options),
      map_cloud(new EllipseLioPointCloud()),
      scan_cloud(new EllipseLioPointCloud()),
      scan_cloud_pub(new EllipseLioPointCloud()),
      harmonics(new EllipsoidHarmonics()),
      kf_(new Ikfom()) {
  this->declare_parameter<int>("mapping.kf_iterations", 1);
  this->declare_parameter<int>("mapping.pub_map_n_secs", 10);
  this->declare_parameter<double>("mapping.map_resolution", 0.1);
  this->declare_parameter<double>("mapping.map_search_radius", 1.0);

  this->declare_parameter<int>("imu.rate", 100);
  this->declare_parameter<double>("imu.gyr_noise", 0.1);
  this->declare_parameter<double>("imu.acc_noise", 0.1);
  this->declare_parameter<double>("imu.gyr_bias", 0.0001);
  this->declare_parameter<double>("imu.acc_bias", 0.0001);
  this->declare_parameter<string>("imu.topic", "");

  this->declare_parameter<int>("lidar.type", 0);
  this->declare_parameter<int>("lidar.rate", 10);
  this->declare_parameter<double>("lidar.min_range", 1.0);
  this->declare_parameter<double>("lidar.max_range", 100.0);
  this->declare_parameter<double>("lidar.bin_size", 1.0);
  this->declare_parameter<double>("lidar.downsample_factor", 0.01);
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

  this->get_parameter_or<int>("mapping.kf_iterations", kf_iterations, 1);
  this->get_parameter_or<int>("mapping.pub_map_n_secs", pub_map_n_secs, 1);
  this->get_parameter_or<double>("mapping.map_resolution", map_resolution, 0.1);
  this->get_parameter_or<double>("mapping.map_search_radius", map_search_radius,
                                 1.0);

  this->get_parameter_or<int>("imu.rate", imu_params.rate, 100);
  this->get_parameter_or<double>("imu.gyr_noise", imu_params.gyr_noise, 0.1);
  this->get_parameter_or<double>("imu.acc_noise", imu_params.acc_noise, 0.1);
  this->get_parameter_or<double>("imu.gyr_bias", imu_params.gyr_bias, 0.0001);
  this->get_parameter_or<double>("imu.acc_bias", imu_params.acc_bias, 0.0001);
  this->get_parameter_or<string>("imu.topic", imu_params.topic, "");

  this->get_parameter_or<int>("lidar.type", lidar_params.type, 0);
  this->get_parameter_or<int>("lidar.rate", lidar_params.rate, 10);
  this->get_parameter_or<string>("lidar.topic", lidar_params.topic, "");
  this->get_parameter_or<double>("lidar.min_range", lidar_params.min_range,
                                 1.0);
  this->get_parameter_or<double>("lidar.max_range", lidar_params.max_range,
                                 100.0);
  this->get_parameter_or<double>("lidar.bin_size", lidar_params.bin_size, 1.0);
  this->get_parameter_or<double>("lidar.downsample_factor",
                                 lidar_params.downsample_factor, 0.01);
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

  map_resolution = fmax(map_resolution, MIN_MAP_RESOLUTION);
  map_search_radius = fmax(map_search_radius, MIN_SEARCH_RADIUS);

  lidar_params.bin_size = fmax(lidar_params.bin_size, MIN_BIN_SIZE);
  lidar_params.map_search_radius = map_search_radius;
  lidar_params.map_resolution = map_resolution;

  map_cloud->reserve(MAX_MAP_POINTS);
  scan_cloud->reserve(MAX_PROC_POINTS);

  ioctree.set_bucket_size(1);
  ioctree.set_min_extent(map_resolution);
  ioctree.set_max_octants(MAX_MAP_POINTS);
  ioctree.set_max_new_points(MAX_PROC_POINTS);

  max_ekfom_time = 0.5 * (1.0 / lidar_params.rate);
  ekfom_data_i = Eigen::ArrayXXi(MAX_PROC_POINTS, 3);
  ekfom_data_c = Eigen::ArrayXXd(MAX_PROC_POINTS, 3);
  ekfom_data_v = Eigen::ArrayXXd(MAX_PROC_POINTS, 3);
  ekfom_data_w = Eigen::ArrayXXd(MAX_PROC_POINTS, 9);
  ekfom_data_h = Eigen::VectorXd(MAX_PROC_POINTS);
  ekfom_data_w_x = Eigen::ArrayXd(MAX_PROC_POINTS);
  ekfom_data_h_x = Eigen::MatrixXd(MAX_PROC_POINTS, 6);
  ekfom_data_h_x_R = Eigen::MatrixXd(6, MAX_PROC_POINTS);

  ekfom_data_h_v =
      std::vector<Eigen::ArrayXd>(3, Eigen::ArrayXd(MAX_PROC_POINTS));
  ekfom_data_h_x_v =
      std::vector<Eigen::MatrixXd>(3, Eigen::MatrixXd(MAX_PROC_POINTS, 6));

  if (num_cams) {
    sh_mats = std::vector<Eigen::MatrixXf>(
        MAX_MAP_POINTS, Eigen::MatrixXf(3, harmonics->getNumCoeffs()));
  }

  colors.reserve(MAX_MAP_POINTS);
  poses.reserve(MAX_MAP_POINTS);
  last_reg.reserve(MAX_MAP_POINTS);
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

  count_reg = std::vector<std::atomic<int>>(MAX_MAP_POINTS);
  updated_pt = std::vector<std::atomic<int>>(MAX_MAP_POINTS);
  new_neighbours_map_idx = std::vector<int>(MAX_SCAN_POINTS);
  new_neighbours_size = std::vector<std::atomic<int>>(MAX_SCAN_POINTS);
  new_neighbours = std::vector<std::vector<int>>(
      MAX_SCAN_POINTS, std::vector<int>(MAX_NEIGHBOURS));

  analytics_msg_ = ellipse_lio::msg::EllipseLioAnalytics();
  analytics_msg_pub_ = ellipse_lio::msg::EllipseLioAnalytics();

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

  double epsi[23] = {0.001};
  kf_->init_dyn_share(get_f, df_dx, df_dw,
                      std::bind(&MappingNode::tensor_registration, this,
                                std::placeholders::_1, std::placeholders::_2),
                      kf_iterations, epsi);

  last_pub_time = rclcpp::Time(0, 0, RCL_ROS_TIME);

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
      this, this->get_clock(), std::chrono::milliseconds(10),
      std::bind(&MappingNode::timer_callback, this), loop_callback_group_);
  pub_odo_timer_ = rclcpp::create_timer(
      this, this->get_clock(),
      std::chrono::milliseconds(1000 / lidar_params.rate),
      std::bind(&MappingNode::publish_odometry, this), pub_odo_callback_group_);
  pub_map_timer_ = rclcpp::create_timer(
      this, this->get_clock(), std::chrono::milliseconds(pub_map_n_secs * 1000),
      std::bind(&MappingNode::publish_map, this), pub_map_callback_group_);

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

// Main mapping loop
void MappingNode::timer_callback() {
  if (!initialized) {
    initialized = true;
    imu_process =
        std::make_shared<ImuProcess>(kf_, imu_params, shared_from_this());
    lid_process =
        std::make_shared<LidarProcess>(lidar_params, shared_from_this());
    init_cam_process();
  }

  if (sync_packages()) {
    double t1, t2, t3, t4, imu_time, state_time, map_time, total_time;
    rclcpp::Time lidar_end_time = rclcpp::Time(0, 0, RCL_ROS_TIME);

    t1 = omp_get_wtime();

    lid_process->GetPointCloud(scan_cloud, lidar_end_time, scan_cloud_bins,
                               start_bin);
    imu_process->UndistortPointCloud(scan_cloud, kf_state_, lidar_end_time,
                                     cams_process);

    if (scan_cloud->empty() || (scan_cloud == NULL)) {
      RCLCPP_WARN(this->get_logger(), "No points skipping scan");
      return;
    }

    t2 = omp_get_wtime();

    if (map_counter) {
      ekfom_iter_cnt = 0;
      ekfom_iter_time = 0;
      imu_process->UpdateStatesWithLidar(kf_state_, lidar_end_time);
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

    analytics_msg_.scan_size = scan_cloud->size();
    analytics_msg_.run_time = t4 - start_time;

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