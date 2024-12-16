#include <map_processing.h>

namespace ellipselivo {

bool MappingNode::sync_packages() {
  if (!imu_process->imu_has_data_) {
    return false;
  }
  if (!lid_process->lidar_has_data_) {
    return false;
  }
  if (imu_process->imu_end_time_ < lid_process->lidar_end_time_) {
    return false;
  }
  if (imu_process->imu_start_time_ > lid_process->lidar_start_time_) {
    lid_process->ClearPointCloud();
    return false;
  }

  return true;
}

void MappingNode::compute_tensor_vote(int i, int j, M3F &A_j, bool first_pass) {
  V3F p_i = map_cloud->points[i].getVector3fMap();
  V3F p_j = map_cloud->points[j].getVector3fMap();
  float d_ij = (p_i - p_j).norm();
  float c_ij = std::exp(-std::pow(d_ij, 2) / search_radius);
  V3F r_ij = (p_i - p_j).normalized();
  M3F rrt = r_ij * r_ij.transpose();
  M3F R_ij = Eye3f - 2.0 * rrt;
  M3F Rp_ij = (Eye3f - 0.5 * rrt) * R_ij;
  M3F K_j = Eye3f;
  if (!first_pass) K_j = tensors_p2[j];
  A_j = c_ij * R_ij * K_j * Rp_ij;
}

void MappingNode::compute_tensor_eigen(int i, M3F &tensor, bool first_pass) {
  V3F eig_val, sali_val;
  M3F eig_vec, tensor_i2;
  Eigen::SelfAdjointEigenSolver<M3F> eig_solver;

  eig_solver.computeDirect(tensor);
  eig_vec = eig_solver.eigenvectors();
  eig_val = eig_solver.eigenvalues().cwiseAbs();

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
    eigenvalues[i] *= search_radius;
    eigenvectors[i] = eig_vec;
    map_cloud->points[i].intensity = (saliency_idxs[i] + 1) * 85;

    if (saliency_idxs[i] == 0) {
      map_cloud->points[i].getNormalVector3fMap() = eig_vec.col(2);
      map_cloud->points[i].curvature = (saliency_idxs[i] + 1) * 85;
    } else if (saliency_idxs[i] == 1) {
      map_cloud->points[i].getNormalVector3fMap() = eig_vec.col(0);
      map_cloud->points[i].curvature = (saliency_idxs[i] + 1) * 85;
    } else if (saliency_idxs[i] == 2) {
      map_cloud->points[i].getNormalVector3fMap() = eig_vec.col(1);
      map_cloud->points[i].curvature = (saliency_idxs[i] + 1) * 85;
    }
  }
}

void MappingNode::tensor_vote_pass_1(int old_map_size,
                                     std::vector<int> &added_idxs,
                                     std::vector<int> &updated_idxs) {
  std::atomic<int> upd_idx = 0, new_neighbours_idx = 0;

#pragma omp parallel for
  for (int i = 0; i < added_idxs.size(); i++) {
    int map_i, loop_cnt;
    Eigen::MatrixXf K;
    std::vector<int> N_idxs;
    M3F tensor_i1;

    map_i = added_idxs[i];
    map_cloud->points[map_i].intensity = 0;
    map_cloud->points[map_i].curvature = 0;
    map_cloud->points[map_i].getNormalVector3fMap() = V3F::Zero();

    ioctree.radiusNeighbors(map_cloud->points[map_i], map_search_radius,
                            N_idxs);
    neighbours[map_i] = N_idxs;

    loop_cnt = min(int(neighbours[map_i].size()), MAX_NEIGHBOURS);
    K = Eigen::MatrixXf::Zero(loop_cnt, 9);

#pragma omp parallel for
    for (int j = 0; j < loop_cnt; j++) {
      M3F A_j;
      int map_j = neighbours[map_i][j];
      compute_tensor_vote(map_i, map_j, A_j, true);
      K.row(j) = A_j.reshaped(1, 9);

      if (!(updated_pt[map_j]++)) {
        update_idx[map_j] = new_neighbours_idx++;
        new_neighbours_map_idx[update_idx[map_j]] = map_j;
        new_neighbours_size[update_idx[map_j]] = 0;
        new_neighbours[update_idx[map_j]]
                      [new_neighbours_size[update_idx[map_j]]++] = map_i;
      } else if (map_j < old_map_size) {
        new_neighbours[update_idx[map_j]]
                      [new_neighbours_size[update_idx[map_j]]++] = map_i;
      }
    }
    tensors_p1[map_i] = K.colwise().sum().reshaped(3, 3);

    filters[map_i][0] = loop_cnt >= NUM_MATCH_POINTS;
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
    if (!updated_pt[map_i]) continue;
    updated_pt[map_i] = 0;

    if (neighbours[map_i].size() >= MAX_NEIGHBOURS) continue;

    updated_idxs[upd_idx++] = map_i;

    max_loop = MAX_NEIGHBOURS - neighbours[map_i].size();
    loop_cnt = min(int(new_neighbours_size[i]), max_loop);

    old_size = neighbours[map_i].size();
    neighbours[map_i].resize(old_size + loop_cnt);

    K = Eigen::MatrixXf::Zero(loop_cnt, 9);

#pragma omp parallel for
    for (int j = 0; j < loop_cnt; j++) {
      M3F A_j;
      int map_j = new_neighbours[i][j];
      neighbours[map_i][old_size + j] = map_j;
      compute_tensor_vote(map_i, map_j, A_j, false);
      K.row(j) = A_j.reshaped(1, 9);
    }

    tensors_p1[map_i] += K.colwise().sum().reshaped(3, 3);

    filters[map_i][0] = neighbours[map_i].size() >= NUM_MATCH_POINTS;
    if (!filters[map_i][0]) continue;

    tensor_i1 = tensors_p1[map_i] / float(neighbours[map_i].size());
    compute_tensor_eigen(map_i, tensor_i1, true);
  }
  updated_idxs.resize(upd_idx);
}

void MappingNode::tensor_vote_pass_2(std::vector<int> &added_idxs,
                                     std::vector<int> &updated_idxs) {
  Eigen::MatrixXf sali_vals;
  Eigen::VectorXi sali_filter;
  V3F cur_mean_sali;

  int total_size = added_idxs.size() + updated_idxs.size();

  sali_vals = Eigen::MatrixXf::Zero(total_size, 3);
  sali_filter = Eigen::VectorXi::Zero(total_size);

#pragma omp parallel for
  for (int i = 0; i < total_size; i++) {
    Eigen::MatrixXf K;
    Eigen::VectorXi K_filter;
    M3F tensor_i2;
    int map_i, loop_cnt, filter_cnt;

    map_i = i < added_idxs.size() ? added_idxs[i]
                                  : updated_idxs[i - added_idxs.size()];
    loop_cnt = min(int(neighbours[map_i].size()), MAX_NEIGHBOURS);

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
    if (filter_cnt < NUM_MATCH_POINTS) continue;

    tensor_i2 = K.colwise().sum().reshaped(3, 3);
    tensor_i2 /= float(filter_cnt);
    compute_tensor_eigen(map_i, tensor_i2, false);

    sali_filter(i) = filters[map_i][1];
    sali_vals.row(i) = salivalues[map_i];
  }

  cur_mean_sali = sali_vals.colwise().sum() / float(sali_filter.sum());
  mean_sali = (cur_mean_sali + (float(map_counter) * mean_sali)) /
              (float(map_counter) + 1);
}

void MappingNode::map_incremental(bool init_map) {
  std::vector<int> added_idxs, new_idxs, updated_idxs;

#pragma omp parallel for
  for (int i = 0; i < scan_cloud->size(); i++) {
    scan_cloud->points[i].getVector3fMap() =
        (kf_state_.state.rot *
             (kf_state_.state.offset_R_L_I *
                  scan_cloud->points[i].getVector3fMap().cast<double>() +
              kf_state_.state.offset_T_L_I) +
         kf_state_.state.pos)
            .cast<float>();
  }

  int old_map_size = map_cloud->size();

  ioctree.set_bucket_size(map_bucket_size);
  ioctree.update(*scan_cloud, added_idxs, new_idxs);
  *map_cloud += EllipseLivoPointCloud(*scan_cloud, added_idxs);

  update_cnt.resize(map_cloud->size(), 0);
  update_idx.resize(map_cloud->size(), 0);
  updated_pt.resize(map_cloud->size(), 0);
  saliency_idxs.resize(map_cloud->size(), 0);
  neighbours.resize(map_cloud->size(), std::vector<int>());
  filters.resize(map_cloud->size(), std::vector<bool>(2, false));

  tensors_p1.resize(map_cloud->size(), M3F::Zero());
  tensors_p2.resize(map_cloud->size(), M3F::Zero());
  salivalues.resize(map_cloud->size(), V3F::Zero());
  eigenvalues.resize(map_cloud->size(), V3F::Zero());
  eigenvectors.resize(map_cloud->size(), M3F::Zero());

  std::cerr << "Map size: " << map_cloud->size() << std::endl;
  std::cerr << "ioctree size: " << ioctree.size() << std::endl;
  std::cerr << "Added idxs size: " << added_idxs.size() << std::endl;

  if (added_idxs.size() > 0) {
    double pass_1_start = omp_get_wtime();
    tensor_vote_pass_1(old_map_size, new_idxs, updated_idxs);
    double pass_1_end = omp_get_wtime();
    std::cerr << "Pass 1 time: " << pass_1_end - pass_1_start << std::endl;
    std::cerr << "Updated idxs size: " << updated_idxs.size() << std::endl;
    tensor_vote_pass_2(new_idxs, updated_idxs);
    double pass_2_end = omp_get_wtime();
    std::cerr << "Pass 2 time: " << pass_2_end - pass_1_end << std::endl;
  }

  map_counter++;
}

void MappingNode::publish_map() {
  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*map_cloud, map_msg);
  map_msg.header.stamp = kf_state_.time;
  map_msg.header.frame_id = "odom_ellipselivo";
  pub_map_->publish(map_msg);
}

void MappingNode::publish_scan() {
  sensor_msgs::msg::PointCloud2 scan_msg;
  pcl::toROSMsg(*scan_cloud, scan_msg);
  scan_msg.header.stamp = kf_state_.time;
  scan_msg.header.frame_id = "odom_ellipselivo";
  pub_scan_->publish(scan_msg);
}

void MappingNode::publish_markers() {
  std::atomic<int> marker_idx = 0;
  int start_idx, end_idx, step_idx, count_idx;
  visualization_msgs::msg::MarkerArray marker_array;

  start_idx = 0;
  end_idx = map_cloud->points.size();
  step_idx = ceil(1e-2 * (end_idx - start_idx));
  count_idx = (end_idx - start_idx) / step_idx;

  marker_array.markers.resize(count_idx);
#pragma omp parallel for
  for (int i = 0; i < count_idx; i++) {
    Eigen::Quaternionf quat;
    visualization_msgs::msg::Marker marker;

    int map_idx = start_idx + (i * step_idx);
    if (!filters[map_idx][1]) continue;

    marker.id = map_idx;
    marker.frame_locked = true;
    marker.ns = "map_primitives";
    marker.lifetime = rclcpp::Duration(0, 0);
    marker.header.frame_id = "odom_ellipselivo";
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
  marker_array.markers.resize(marker_idx);
  pub_mark_->publish(marker_array);
}

void MappingNode::publish_odometry() {
  // pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

  geometry_msgs::msg::TransformStamped trans;
  trans.header.frame_id = "odom_ellipselivo";
  trans.child_frame_id = "imu_ellipselivo";
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

void MappingNode::tensor_registration(
    state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
  Eigen::MatrixXd h(scan_cloud->size(), 1);
  Eigen::MatrixXd h_x(scan_cloud->size(), 12);
  std::atomic<int> feat_cnt = 0, plane_cnt = 0, curve_cnt = 0, junct_cnt = 0;

  double res_mean_last = 0.0, total_residual = 0.0;
  double match_start = omp_get_wtime();

#pragma omp parallel for
  for (int i = 0; i < scan_cloud->size(); i++) {
    int sali_idx, map_i;
    float residual;
    V3F c, a;
    M3F P_skew;
    std::vector<int> N_idxs, N_p_idxs;
    std::vector<float> N_dst, N_p_dst;
    V3F p_lidar, p_imu, p_world;
    V3F sali, n_world, p_dash, q, q_dash, norm_vec, eig_vals;

    EllipseLivoPoint pt = scan_cloud->points[i];

    p_lidar = pt.getVector3fMap();
    p_imu = (s.offset_R_L_I * p_lidar.cast<double>() + s.offset_T_L_I)
                .cast<float>();
    p_world =
        (s.rot * (s.offset_R_L_I * p_lidar.cast<double>() + s.offset_T_L_I) +
         s.pos)
            .cast<float>();

    pt.getVector3fMap() = p_world;
    ioctree.knnNeighbors(pt, 1, N_idxs, N_dst);
    map_i = N_idxs[0];
    if (sqrt(N_dst[0]) > map_search_radius || !filters[map_i][1]) continue;

    sali_idx = saliency_idxs[map_i];
    if (salivalues[map_i](sali_idx) < mean_sali(sali_idx)) continue;

    n_world = map_cloud->points[map_i].getVector3fMap();

    eig_vals = eigenvalues[map_i];

    if (sali_idx == 0) {
      // Point to plane
      q = p_world - n_world;
      q_dash = q.dot(eigenvectors[map_i].col(2)) * eigenvectors[map_i].col(2);
      p_dash = p_world - q_dash;
      norm_vec = p_world - p_dash;
      p_dash = eigenvectors[map_i].transpose() * (p_dash - n_world);
      if (p_dash.cwiseQuotient(eig_vals).cwiseAbs2().sum() > 1.0) continue;
      plane_cnt++;
    } else if (sali_idx == 1) {
      //  Point to curve
      q = p_world - n_world;
      q_dash = q.dot(eigenvectors[map_i].col(0)) * eigenvectors[map_i].col(0);
      p_dash = n_world + q_dash;
      norm_vec = p_world - p_dash;
      p_dash = eigenvectors[map_i].transpose() * (p_dash - n_world);
      if (p_dash.cwiseQuotient(eig_vals).cwiseAbs2().sum() > 1.0) continue;
      curve_cnt++;
    } else if (sali_idx == 2) {
      //  Point to junction
      norm_vec = p_world - n_world;
      p_dash = eigenvectors[map_i].transpose() * (p_world - n_world);
      if (p_dash.cwiseQuotient(eig_vals).cwiseAbs2().sum() > 1.0) continue;
      junct_cnt++;
    }

    residual = norm_vec.norm();
    norm_vec.normalize();

    P_skew << SKEW_SYM_MATRX(p_imu);

    c = s.rot.conjugate().cast<float>() * norm_vec;
    a = P_skew * c;

    int feat_num = ++feat_cnt;
    h_x.row(feat_num - 1) << norm_vec(0), norm_vec(1), norm_vec(2),
        VEC_FROM_ARRAY(a), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    h(feat_num - 1) = -residual;

    total_residual += residual;
  }

  h.conservativeResize(feat_cnt, 1);
  h_x.conservativeResize(feat_cnt, 12);

  ekfom_data.h = h;
  ekfom_data.h_x = h_x;

  res_mean_last = total_residual / feat_cnt;

  std::cerr << "Res mean: " << res_mean_last << std::endl;
  std::cerr << "Num feats: " << feat_cnt << std::endl;
  std::cerr << "Num planes: " << plane_cnt << std::endl;
  std::cerr << "Num curves: " << curve_cnt << std::endl;
  std::cerr << "Num junctions: " << junct_cnt << std::endl;

  match_time += omp_get_wtime() - match_start;
}

MappingNode::MappingNode(
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node("laser_mapping", options),
      map_cloud(new EllipseLivoPointCloud()),
      scan_cloud(new EllipseLivoPointCloud()),
      kf_(new Ikfom()) {
  this->declare_parameter<int>("publish.pub_map_n_secs", 10);

  this->declare_parameter<int>("mapping.kf_iterations", 1);
  this->declare_parameter<double>("mapping.map_resolution", 0.1);
  this->declare_parameter<double>("mapping.map_search_radius", 1.0);

  this->declare_parameter<int>("imu.rate", 100);
  this->declare_parameter<double>("imu.gyr_noise", 0.1);
  this->declare_parameter<double>("imu.acc_noise", 0.1);
  this->declare_parameter<double>("imu.gyr_bias", 0.0001);
  this->declare_parameter<double>("imu.acc_bias", 0.0001);
  this->declare_parameter<string>("imu.topic", "/livox/imu");

  this->declare_parameter<int>("lidar.type", 0);
  this->declare_parameter<int>("lidar.rate", 10);
  this->declare_parameter<double>("lidar.min_range", 1.0);
  this->declare_parameter<double>("lidar.max_range", 100.0);
  this->declare_parameter<double>("lidar.bin_size", 1.0);
  this->declare_parameter<double>("lidar.downsample_factor", 0.01);
  this->declare_parameter<string>("lidar.topic", "/livox/lidar");
  this->declare_parameter<vector<double>>("lidar.t_imu_lidar",
                                          vector<double>());
  this->declare_parameter<vector<double>>("lidar.r_imu_lidar",
                                          vector<double>());

  this->declare_parameter<int>("cameras.frame_rate", 20);
  this->declare_parameter<vector<string>>("cameras.cam_topics",
                                          vector<string>());
  this->declare_parameter<vector<double>>("cameras.cam_intrinsics",
                                          vector<double>());
  this->declare_parameter<vector<double>>("cameras.T_cam_lidars",
                                          vector<double>());
  this->declare_parameter<vector<double>>("cameras.R_cam_lidars",
                                          vector<double>());

  this->get_parameter_or<int>("publish.pub_map_n_secs", pub_map_n_secs, 1);

  this->get_parameter_or<int>("mapping.kf_iterations", kf_iterations, 1);
  this->get_parameter_or<double>("mapping.map_resolution", map_resolution, 0.1);
  this->get_parameter_or<double>("mapping.map_search_radius", search_radius,
                                 1.0);

  this->get_parameter_or<int>("imu.rate", imu_params.rate, 100);
  this->get_parameter_or<double>("imu.gyr_noise", imu_params.gyr_noise, 0.1);
  this->get_parameter_or<double>("imu.acc_noise", imu_params.acc_noise, 0.1);
  this->get_parameter_or<double>("imu.gyr_bias", imu_params.gyr_bias, 0.0001);
  this->get_parameter_or<double>("imu.acc_bias", imu_params.acc_bias, 0.0001);
  this->get_parameter_or<string>("imu.topic", imu_params.topic, "/livox/imu");

  this->get_parameter_or<int>("lidar.type", lidar_params.type, 0);
  this->get_parameter_or<int>("lidar.rate", lidar_params.rate, 10);
  this->get_parameter_or<string>("lidar.topic", lidar_params.topic,
                                 "/livox/lidar");
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

  this->get_parameter_or<int>("cameras.frame_rate", cam_frame_rate, 20);
  this->get_parameter_or<vector<string>>("cameras.cam_topics", cam_topics,
                                         vector<string>());
  this->get_parameter_or<vector<double>>("cameras.cam_intrinsics",
                                         cam_intrinsics, vector<double>());
  this->get_parameter_or<vector<double>>("cameras.T_cam_lidars", T_cam_lidars,
                                         vector<double>());
  this->get_parameter_or<vector<double>>("cameras.R_cam_lidars", R_cam_lidars,
                                         vector<double>());

  ioctree.set_min_extent(map_resolution);
  ioctree.set_bucket_size(1);

  mean_sali = V3F::Zero();

  new_neighbours_map_idx = std::vector<int>(100000);
  new_neighbours_size = std::vector<std::atomic<int>>(100000);
  new_neighbours =
      std::vector<std::vector<int>>(100000, std::vector<int>(1000));

  imu_params.t_imu_lidar << VEC_FROM_ARRAY(t_imu_lidar);
  imu_params.r_imu_lidar << MAT_FROM_ARRAY(r_imu_lidar);

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

// void MappingNode::init_cam_process() {
//   if (cam_init) {
//     return;
//   }
//   cam_init = true;
//   if (cam_topics.empty()) {
//     RCLCPP_INFO(this->get_logger(), "No camera topics, skip camera process");
//     return;
//   }
//   if (cam_topics.size() * 3 != T_cam_lidars.size()) {
//     RCLCPP_ERROR(this->get_logger(),
//                  "The number of camera topics and T_cam_lidars are not "
//                  "consistent, skip camera process");
//     return;
//   }
//   if (cam_topics.size() * 9 != R_cam_lidars.size()) {
//     RCLCPP_ERROR(this->get_logger(),
//                  "The number of camera topics and R_cam_lidars are not "
//                  "consistent, skip camera process");
//     return;
//   }
//   if (cam_topics.size() * 9 != cam_intrinsics.size()) {
//     RCLCPP_ERROR(this->get_logger(),
//                  "The number of camera topics and cam_intrinsics are not "
//                  "consistent, skip camera process");
//     return;
//   }
//   for (int i = 0; i < cam_topics.size(); i++) {
//     p_cams.push_back(std::make_shared<CamProcess>(cam_frame_rate,
//     cam_topics[i],
//                                                   shared_from_this()));
//     V3D Lidar_T_wrt_Cam(Zero3d);
//     M3D Lidar_R_wrt_Cam(Eye3d);
//     M3D cam_intrinsic_mat(Eye3d);
//     vector<double> T_cam_lidar(T_cam_lidars.begin() + i * 3,
//                                T_cam_lidars.begin() + i * 3 + 3);
//     vector<double> R_cam_lidar(R_cam_lidars.begin() + i * 9,
//                                R_cam_lidars.begin() + i * 9 + 9);
//     vector<double> cam_intrinsic(cam_intrinsics.begin() + i * 9,
//                                  cam_intrinsics.begin() + i * 9 + 9);
//     Lidar_T_wrt_Cam << VEC_FROM_ARRAY(T_cam_lidar);
//     Lidar_R_wrt_Cam << MAT_FROM_ARRAY(R_cam_lidar);
//     cam_intrinsic_mat << MAT_FROM_ARRAY(cam_intrinsic);
//     p_cams[i]->SetExtrinsicAndIntrinsic(Lidar_T_wrt_Cam, Lidar_R_wrt_Cam,
//                                         Lidar_T_wrt_IMU, Lidar_R_wrt_IMU,
//                                         cam_intrinsic_mat);
//   }
// }

void MappingNode::timer_callback() {
  //  init_cam_process();

  if (!initialized) {
    initialized = true;
    imu_process =
        std::make_shared<ImuProcess>(kf_, imu_params, shared_from_this());
    lid_process =
        std::make_shared<LidarProcess>(lidar_params, shared_from_this());
  }

  if (sync_packages()) {
    std::cerr << "Synced packages" << std::endl;
    double t0, t1, t2, t3, t4, t5, t6, t7, solve_time;
    rclcpp::Time lidar_end_time = rclcpp::Time(0, 0, RCL_ROS_TIME);

    t0 = omp_get_wtime();

    lid_process->GetPointCloud(scan_cloud, lidar_end_time);
    imu_process->UndistortPointCloud(scan_cloud, kf_state_, lidar_end_time);

    t1 = omp_get_wtime();
    imu_time = t1 - t0;

    if (scan_cloud->empty() || (scan_cloud == NULL)) {
      RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
      return;
    }

    map_bucket_size = 1;
    map_search_radius = search_radius;

    std::cerr << "Bucket size: " << map_bucket_size << std::endl;
    std::cerr << "Search radius: " << map_search_radius << std::endl;

    if (ioctree.size() == 0) {
      RCLCPP_INFO(this->get_logger(), "Initialize the map kdtree");
      if (scan_cloud->points.size() < NUM_MATCH_POINTS) return;
      map_incremental(true);
      return;
    }

    std::cerr << "Scan size: " << scan_cloud->size() << std::endl;

    t2 = omp_get_wtime();
    downsample_time = t2 - t1;

    t4 = omp_get_wtime();
    imu_process->UpdateStatesWithLidar(solve_time, kf_state_, lidar_end_time);
    t5 = omp_get_wtime();

    state_update_time = t5 - t4;
    map_incremental(false);
    t6 = omp_get_wtime();
    map_update_time = t6 - t5;
    total_time = t6 - t0;

    publish_odometry();
    publish_scan();

    max_time_match = fmax(max_time_match, match_time);
    max_time_solve = fmax(max_time_solve, solve_time);
    max_imu_time = fmax(max_imu_time, imu_time);
    max_state_update_time = fmax(max_state_update_time, state_update_time);
    max_map_update_time = fmax(max_map_update_time, map_update_time);
    max_downsample_time = fmax(max_downsample_time, downsample_time);
    max_total_time = fmax(max_total_time, total_time);
    printf(
        "IMU: %0.6f Downsample: %0.6f Match time: %0.6f "
        "State update: %0.6f Map update: %0.6f "
        "Total: %0.6f\n",
        imu_time, downsample_time, match_time, state_update_time,
        map_update_time, total_time);
    printf(
        "Max IMU: %0.6f Max downsample: %0.6f Max match time: %0.6f "
        "Max state update: %0.6f Max map update: "
        "%0.6f Max total time: %0.6f\n",
        max_imu_time, max_downsample_time, max_time_match,
        max_state_update_time, max_map_update_time, max_total_time);
  }
}

}  // namespace ellipselivo

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ellipselivo::MappingNode)