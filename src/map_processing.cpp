#include <map_processing.h>

namespace ellipselio {

// Sync lida, imu, and camera data
bool MappingNode::sync_packages() {
  double inter_sync_time = omp_get_wtime() - last_sync_time;

  if (!last_sync_time) {
    if (int(floor(inter_sync_time / 0.01)) % 100 == 0) {
      RCLCPP_INFO(this->get_logger(), "Waiting for data...");
    }
  }
  if (!imu_process->imu_has_data_) {
    if (int(floor(inter_sync_time / 0.01)) % 10 == 0) {
      RCLCPP_ERROR(this->get_logger(), "IMU has no data");
    }
    return false;
  }
  if (!lid_process->lidar_has_data_) {
    if (int(floor(inter_sync_time / 0.01)) % 10 == 0) {
      RCLCPP_ERROR(this->get_logger(), "Lidar has no data");
    }
    return false;
  }
  if (imu_process->imu_end_time_ < lid_process->lidar_end_time_) {
    if (int(floor(inter_sync_time / 0.01)) % 10 == 0) {
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
    if (int(floor(inter_sync_time / 0.01)) % 10 == 0) {
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
      if (int(floor(inter_sync_time / 0.01)) % 10 == 0) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Camera " << i << " has no data");
      }
      return false;
    }
  }
  for (int i = 0; i < num_cams; i++) {
    if (cams_process[i]->img_end_time_ < imu_process->imu_start_time_) {
      if (int(floor(inter_sync_time / 0.01)) % 10 == 0) {
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

  RCLCPP_INFO_STREAM(this->get_logger(), "Imu freq: " << cur_imu_freq);
  RCLCPP_INFO_STREAM(this->get_logger(), "Lid freq: " << cur_lid_freq);

  for (int i = 0; i < num_cams; i++) {
    int cur_cam_freq = round(cams_process[i]->cam_counter_ / inter_sync_time);
    cams_process[i]->cam_counter_ = 0;
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Cam " << i << " freq: " << cur_cam_freq);
  }

  int cur_odom_freq = round(1.0 / inter_sync_time);
  RCLCPP_INFO_STREAM(this->get_logger(), "Odom freq: " << cur_odom_freq);

  RCLCPP_INFO(this->get_logger(), "Synced packages");
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
    eigenvalues[i] = (1.0 / (eig_val.array() + 1e-3)).matrix().normalized();
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
    updated_pt[map_i] = 0;
    map_cloud->points[map_i].intensity = 0;

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
  std::vector<Eigen::MatrixXf> sali_vals;
  std::vector<Eigen::VectorXi> sali_filter;

  int total_size = added_idxs.size() + updated_idxs.size();

  sali_vals = std::vector<Eigen::MatrixXf>(
      num_bins, Eigen::MatrixXf::Zero(total_size, 3));
  sali_filter =
      std::vector<Eigen::VectorXi>(num_bins, Eigen::VectorXi::Zero(total_size));

#pragma omp parallel for
  for (int i = 0; i < total_size; i++) {
    V3F old_sali;
    bool old_filter;
    Eigen::MatrixXf K;
    Eigen::VectorXi K_filter;
    M3F tensor_i2;
    int map_i, loop_cnt, filter_cnt;

    map_i = i < added_idxs.size() ? added_idxs[i]
                                  : updated_idxs[i - added_idxs.size()];

    const int &bin_idx = map_cloud->points[map_i].bin_idx;
    const int &min_neigh = lid_process->min_neighbours_[bin_idx];
    const int &max_neigh = lid_process->max_neighbours_[bin_idx];
    loop_cnt = min(int(neighbours[map_i].size()), max_neigh);

    if (!filters[map_i][0]) continue;

    K = Eigen::MatrixXf::Zero(loop_cnt, 9);
    K_filter = Eigen::VectorXi::Zero(loop_cnt);

#pragma omp parallel for
    for (int j = 0; j < loop_cnt; j++) {
      int map_j = neighbours[map_i][j];
      if (!filters[map_j][0]) continue;

      M3F A_j;
      compute_tensor_vote(map_i, map_j, A_j, false);
      K.row(j) = A_j.reshaped(1, 9);
      K_filter(j) = 1;
    }

    filter_cnt = K_filter.sum();
    if (filter_cnt < min_neigh) continue;

    old_sali = salivalues[map_i];
    old_filter = filters[map_i][1];

    tensor_i2 = K.colwise().sum().reshaped(3, 3);
    tensor_i2 /= float(filter_cnt);
    compute_tensor_eigen(map_i, tensor_i2, false);

    sali_filter[bin_idx](i) = filters[map_i][1] && !old_filter;
    sali_vals[bin_idx].row(i) = salivalues[map_i] - old_sali;
  }

#pragma omp parallel for
  for (int i = 0; i < num_bins; i++) {
    Eigen::Vector3f sali_vals_sum = sali_vals[i].colwise().sum();
    if (sali_vals_sum.sum() == 0) continue;
    mean_sali[i] = ((mean_cnt[i] * mean_sali[i]) + sali_vals_sum) /
                   (mean_cnt[i] + sali_filter[i].sum());
    mean_cnt[i] += sali_filter[i].sum();
  }
}

// Add new points to the map and update geometric primitives
void MappingNode::map_incremental(bool init_map) {
  int start_idx, end_idx;
  std::vector<int> new_idxs, updated_idxs, added_idxs_i, new_idxs_i;

#pragma omp parallel for
  for (int i = 0; i < scan_cloud->size(); i++) {
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
    if (!scan_cloud_bins[i]) continue;

    end_idx += scan_cloud_bins[i];
    ioctree.set_bucket_size(lid_process->bucket_sizes_[fmax(i, start_bin)]);
    ioctree.update(*scan_cloud, added_idxs_i, new_idxs_i, true, start_idx,
                   end_idx);
    *map_cloud += EllipseLioPointCloud(*scan_cloud, added_idxs_i);
    new_idxs.insert(new_idxs.end(), new_idxs_i.begin(), new_idxs_i.end());
    start_idx = end_idx;
  }
  new_map_size = map_cloud->size();

  update_idx.resize(map_cloud->size(), 0);
  saliency_idxs.resize(map_cloud->size(), 0);
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

  RCLCPP_INFO_STREAM(this->get_logger(), "Map size: " << map_cloud->size());
  RCLCPP_INFO_STREAM(this->get_logger(), "Oct num: " << ioctree.octant_size());
  RCLCPP_INFO_STREAM(this->get_logger(), "New idxs: " << new_idxs.size());
  RCLCPP_INFO_STREAM(this->get_logger(), "Upd idxs: " << updated_idxs.size());

  map_counter++;
}

// Publish map point cloud
void MappingNode::publish_map() {
  sensor_msgs::msg::PointCloud2 map_msg;

  if (!map_cloud->size()) return;
  pcl::toROSMsg(*map_cloud, map_msg);
  map_msg.header.stamp = kf_state_.time;
  map_msg.header.frame_id = "odom_ellipselio";
  pub_map_->publish(map_msg);
}

// Publish scan point cloud
void MappingNode::publish_scan() {
  sensor_msgs::msg::PointCloud2 scan_msg;
  pcl::toROSMsg(*scan_cloud, scan_msg);
  scan_msg.header.stamp = kf_state_.time;
  scan_msg.header.frame_id = "odom_ellipselio";
  pub_scan_->publish(scan_msg);
}

// Publish geometric primitive markers
void MappingNode::publish_markers() {
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
    marker.header.stamp = kf_state_.time;
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
  geometry_msgs::msg::TransformStamped trans;
  trans.header.frame_id = "odom_ellipselio";
  trans.child_frame_id = "imu_ellipselio";
  trans.header.stamp = kf_state_.time;
  trans.transform.translation.x = kf_state_.state.pos(0);
  trans.transform.translation.y = kf_state_.state.pos(1);
  trans.transform.translation.z = kf_state_.state.pos(2);
  trans.transform.rotation.x = kf_state_.state.rot.coeffs()[0];
  trans.transform.rotation.y = kf_state_.state.rot.coeffs()[1];
  trans.transform.rotation.z = kf_state_.state.rot.coeffs()[2];
  trans.transform.rotation.w = kf_state_.state.rot.coeffs()[3];
  tf_br_->sendTransform(trans);
}

// Register new scan points to the map using tensor registration
void MappingNode::tensor_registration(
    state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
  std::atomic<int> feat_cnt = 0, plane_cnt = 0, curve_cnt = 0, junct_cnt = 0;

  if (ekfom_iter_cnt > 0) {
    if (max_ekfom_time - ekfom_iter_time < ekfom_iter_time / ekfom_iter_cnt) {
      ekfom_data.valid = false;
      return;
    }
  }

  double t0 = omp_get_wtime();

#pragma omp parallel for
  for (int i = 0; i < scan_cloud->size(); i++) {
    float residual, score;
    int sali_idx, map_i, feat_num;
    std::vector<int> N_idxs, N_p_idxs;
    std::vector<float> N_dst, N_p_dst;

    V3F a;
    V3D p_imu;
    M3F P_skew;
    V3F p_lidar, p_world;
    V3F sali, n_world, p_dash, q, q_dash, norm_vec, eig_vals;

    const EllipseLioPoint &pt = scan_cloud->points[i];

    p_lidar = pt.getVector3fMap();
    p_imu = (s.offset_R_L_I * p_lidar.cast<double>() + s.offset_T_L_I);
    p_world = (s.rot * p_imu + s.pos).cast<float>();

    const int scan_bin_idx = fmax(scan_cloud->points[i].bin_idx, start_bin);
    const float &search_rad = lid_process->search_radii_[scan_bin_idx];

    ioctree.knnNeighbors(p_world, 1, N_idxs, N_dst, search_rad);
    if (N_idxs.size() == 0) continue;
    map_i = N_idxs[0];

    if (!filters[map_i][1]) continue;

    const int &map_bin_idx = map_cloud->points[map_i].bin_idx;

    sali_idx = saliency_idxs[map_i];
    // if (salivalues[map_i](sali_idx) < mean_sali[map_bin_idx](sali_idx))
    //   continue;

    n_world = map_cloud->points[map_i].getVector3fMap();
    eig_vals = eigenvalues[map_i];

    if (sali_idx == 0) {
      // Point to plane
      q = p_world - n_world;
      q_dash = q.dot(eigenvectors[map_i].col(2)) * eigenvectors[map_i].col(2);
      p_dash = p_world - q_dash;
      norm_vec = p_world - p_dash;
      p_dash = eigenvectors[map_i].transpose() * (p_dash - n_world);
      score = 1.0 - (eig_vals(2) / eig_vals.sum());
      // std::cerr << "Plane score: " << score << std::endl;
      if (score < 0.9) continue;
      // if (p_dash.cwiseQuotient(eig_vals).cwiseAbs2().sum() > 1.0) continue;
      plane_cnt++;
    } else if (sali_idx == 1) {
      //  Point to line
      q = p_world - n_world;
      q_dash = q.dot(eigenvectors[map_i].col(0)) * eigenvectors[map_i].col(0);
      p_dash = n_world + q_dash;
      norm_vec = p_world - p_dash;
      p_dash = eigenvectors[map_i].transpose() * (p_dash - n_world);
      score = (eig_vals(0) - eig_vals(1)) / eig_vals(0);
      // std::cerr << "Line score: " << score << std::endl;
      if (score < 0.9) continue;
      // if (p_dash.cwiseQuotient(eig_vals).cwiseAbs2().sum() > 1.0) continue;
      curve_cnt++;
    } else if (sali_idx == 2) {
      //  Point to point
      norm_vec = p_world - n_world;
      p_dash = eigenvectors[map_i].transpose() * (p_world - n_world);
      score = 1 - ((eig_vals(0) - eig_vals(2)) / eig_vals.sum());
      // std::cerr << "Point score: " << score << std::endl;
      if (score < 0.9) continue;
      // if (p_dash.cwiseQuotient(eig_vals).cwiseAbs2().sum() > 1.0) continue;
      junct_cnt++;
    }

    residual = norm_vec.norm();
    norm_vec.normalize();

    P_skew << SKEW_SYM_MATRIX(p_imu.cast<float>());
    a = P_skew * s.rot.conjugate().cast<float>() * norm_vec;

    feat_num = ++feat_cnt;
    ekfom_data_h(feat_num - 1) = -residual;
    ekfom_data_h_x.row(feat_num - 1) << norm_vec(0), norm_vec(1), norm_vec(2),
        VEC_FROM_ARRAY(a), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  }

  ekfom_data.h = ekfom_data_h.head(int(feat_cnt));
  ekfom_data.h_x = ekfom_data_h_x.topRows(int(feat_cnt));

  double res_mean = -ekfom_data.h.sum() / feat_cnt;
  double t1 = omp_get_wtime();

  ekfom_iter_cnt++;
  ekfom_iter_time += t1 - t0;

  RCLCPP_INFO_STREAM(this->get_logger(), "Num feats: " << feat_cnt);
  RCLCPP_INFO_STREAM(this->get_logger(), "Num planes: " << plane_cnt);
  RCLCPP_INFO_STREAM(this->get_logger(), "Num curves: " << curve_cnt);
  RCLCPP_INFO_STREAM(this->get_logger(), "Num junctions: " << junct_cnt);
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Res mean: " << std::setprecision(2) << res_mean);
}

// Main mapping node
MappingNode::MappingNode(
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node("mapping_node", options),
      map_cloud(new EllipseLioPointCloud()),
      scan_cloud(new EllipseLioPointCloud()),
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
  scan_cloud->reserve(MAX_SCAN_POINTS);

  ioctree.set_bucket_size(1);
  ioctree.set_min_extent(map_resolution);
  ioctree.set_max_octants(MAX_MAP_POINTS);
  ioctree.set_max_new_points(MAX_SCAN_POINTS);

  max_ekfom_time = 0.5 * (1.0 / lidar_params.rate);
  ekfom_data_h = Eigen::VectorXd(MAX_SCAN_POINTS, 1);
  ekfom_data_h_x = Eigen::MatrixXd(MAX_SCAN_POINTS, 12);

  update_idx.reserve(MAX_MAP_POINTS);
  saliency_idxs.reserve(MAX_MAP_POINTS);
  neighbours.reserve(MAX_MAP_POINTS);
  filters.reserve(MAX_MAP_POINTS);

  tensors_p1.reserve(MAX_MAP_POINTS);
  tensors_p2.reserve(MAX_MAP_POINTS);
  salivalues.reserve(MAX_MAP_POINTS);
  eigenvalues.reserve(MAX_MAP_POINTS);
  eigenvectors.reserve(MAX_MAP_POINTS);

  num_bins = ceil(lidar_params.max_range / lidar_params.bin_size);
  mean_cnt = std::vector<int>(num_bins, 1);
  mean_sali = std::vector<V3F>(num_bins, V3F::Zero());

  updated_pt = std::vector<std::atomic<int>>(MAX_MAP_POINTS);
  new_neighbours_map_idx = std::vector<int>(MAX_SCAN_POINTS);
  new_neighbours_size = std::vector<std::atomic<int>>(MAX_SCAN_POINTS);
  new_neighbours = std::vector<std::vector<int>>(
      MAX_SCAN_POINTS, std::vector<int>(MAX_NEIGHBOURS));

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

  loop_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pub_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  pub_map_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_map", 1);
  pub_scan_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_scan", 1);
  pub_mark_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/visualization_marker", 1);

  loop_timer_ = rclcpp::create_timer(
      this, this->get_clock(), std::chrono::milliseconds(10),
      std::bind(&MappingNode::timer_callback, this), loop_callback_group_);
  pub_map_timer_ = rclcpp::create_timer(
      this, this->get_clock(), std::chrono::milliseconds(pub_map_n_secs * 1000),
      std::bind(&MappingNode::publish_map, this), pub_callback_group_);
  pub_marker_timer_ = rclcpp::create_timer(
      this, this->get_clock(), std::chrono::milliseconds(pub_map_n_secs * 1000),
      std::bind(&MappingNode::publish_markers, this), pub_callback_group_);

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
    double t0, t1, t2, t3, t4, t5, imu_time, state_time, map_time, pub_time,
        total_time;
    rclcpp::Time lidar_end_time = rclcpp::Time(0, 0, RCL_ROS_TIME);

    t0 = omp_get_wtime();

    lid_process->GetPointCloud(scan_cloud, lidar_end_time, scan_cloud_bins,
                               start_bin);
    imu_process->UndistortPointCloud(scan_cloud, kf_state_, lidar_end_time,
                                     cams_process);

    t1 = omp_get_wtime();

    if (scan_cloud->empty() || (scan_cloud == NULL)) {
      RCLCPP_WARN(this->get_logger(), "No points skipping scan");
      return;
    }

    if (ioctree.size() == 0) {
      RCLCPP_INFO(this->get_logger(), "Initialize the map");
      map_incremental(true);
      return;
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Scan size: " << scan_cloud->size());

    ekfom_iter_cnt = 0;
    ekfom_iter_time = 0;
    t2 = omp_get_wtime();
    imu_process->UpdateStatesWithLidar(kf_state_, lidar_end_time);

    t3 = omp_get_wtime();
    map_incremental(false);

    t4 = omp_get_wtime();
    publish_odometry();
    publish_scan();
    t5 = omp_get_wtime();

    imu_time = t1 - t0;
    state_time = t3 - t2;
    map_time = t4 - t3;
    pub_time = t5 - t4;
    total_time = t5 - t0;

    max_imu_time = fmax(max_imu_time, imu_time);
    max_state_time = fmax(max_state_time, state_time);
    max_map_time = fmax(max_map_time, map_time);
    max_total_time = fmax(max_total_time, total_time);

    mean_imu_time += imu_time;
    mean_state_time += state_time;
    mean_map_time += map_time;
    mean_total_time += total_time;

    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO_STREAM(this->get_logger(), "Imu: " << std::fixed
                                                   << std::setprecision(3)
                                                   << imu_time);
    RCLCPP_INFO_STREAM(this->get_logger(), "State: " << std::fixed
                                                     << std::setprecision(3)
                                                     << state_time);
    RCLCPP_INFO_STREAM(this->get_logger(), "Map: " << std::fixed
                                                   << std::setprecision(3)
                                                   << map_time);
    RCLCPP_INFO_STREAM(this->get_logger(), "Pub: " << std::fixed
                                                   << std::setprecision(3)
                                                   << pub_time);
    RCLCPP_INFO_STREAM(this->get_logger(), "Total: " << std::fixed
                                                     << std::setprecision(3)
                                                     << total_time);
    RCLCPP_INFO(this->get_logger(), " ");

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Mean Imu: " << std::fixed << std::setprecision(3)
                                    << mean_imu_time / (map_counter - 1));
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Mean State: " << std::fixed << std::setprecision(3)
                                      << mean_state_time / (map_counter - 1));
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Mean Map: " << std::fixed << std::setprecision(3)
                                    << mean_map_time / (map_counter - 1));
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Mean Total: " << std::fixed << std::setprecision(3)
                                      << mean_total_time / (map_counter - 1));
    RCLCPP_INFO(this->get_logger(), " ");

    RCLCPP_INFO_STREAM(this->get_logger(), "Max imu: " << std::fixed
                                                       << std::setprecision(3)
                                                       << max_imu_time);
    RCLCPP_INFO_STREAM(this->get_logger(), "Max state: " << std::fixed
                                                         << std::setprecision(3)
                                                         << max_state_time);
    RCLCPP_INFO_STREAM(this->get_logger(), "Max map: " << std::fixed
                                                       << std::setprecision(3)
                                                       << max_map_time);
    RCLCPP_INFO_STREAM(this->get_logger(), "Max total: " << std::fixed
                                                         << std::setprecision(3)
                                                         << max_total_time);
    RCLCPP_INFO(this->get_logger(), " ");
  }
}

}  // namespace ellipselio

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ellipselio::MappingNode)