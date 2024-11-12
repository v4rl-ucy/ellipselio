// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <laser_mapping.h>

namespace fastlio {

void LaserMappingNode::dump_lio_state_to_log(FILE *fp) {
  V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
  fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
  fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));  // Angle
  fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1),
          state_point.pos(2));                 // Pos
  fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);  // omega
  fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1),
          state_point.vel(2));                 // Vel
  fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);  // Acc
  fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1),
          state_point.bg(2));  // Bias_g
  fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1),
          state_point.ba(2));  // Bias_a
  fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1],
          state_point.grav[2]);  // Bias_a
  fprintf(fp, "\r\n");
  fflush(fp);
}

void LaserMappingNode::pointLidarToWorld_ikfom(FastLioPoint const *const pi,
                                               FastLioPoint *const po,
                                               state_ikfom &s) {
  V3D p_lidar(pi->x, pi->y, pi->z);
  V3D p_global(s.rot * (s.offset_R_L_I * p_lidar + s.offset_T_L_I) + s.pos);

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->r = pi->r;
  po->g = pi->g;
  po->b = pi->b;
  po->intensity = pi->intensity;
  po->offset_time = pi->offset_time;
  po->has_color = pi->has_color;
}

void LaserMappingNode::pointLidarToIMU_ikfom(FastLioPoint const *const pi,
                                             FastLioPoint *const po,
                                             state_ikfom &s) {
  V3D p_lidar(pi->x, pi->y, pi->z);
  V3D p_imu(s.offset_R_L_I * p_lidar + s.offset_T_L_I);

  po->x = p_imu(0);
  po->y = p_imu(1);
  po->z = p_imu(2);
  po->r = pi->r;
  po->g = pi->g;
  po->b = pi->b;
  po->intensity = pi->intensity;
  po->offset_time = pi->offset_time;
  po->has_color = pi->has_color;
}

void LaserMappingNode::pointLidarToWorld(FastLioPoint const *const pi,
                                         FastLioPoint *const po) {
  V3D p_lidar(pi->x, pi->y, pi->z);
  V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_lidar +
                                  state_point.offset_T_L_I) +
               state_point.pos);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->r = pi->r;
  po->g = pi->g;
  po->b = pi->b;
  po->intensity = pi->intensity;
  po->offset_time = pi->offset_time;
  po->has_color = pi->has_color;
}

template <typename T>
void LaserMappingNode::pointLidarToWorld(const Matrix<T, 3, 1> &pi,
                                         Matrix<T, 3, 1> &po) {
  V3D p_lidar(pi[0], pi[1], pi[2]);
  V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_lidar +
                                  state_point.offset_T_L_I) +
               state_point.pos);

  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

void LaserMappingNode::RGBpointLidarToWorld(FastLioPoint const *const pi,
                                            FastLioPoint *const po) {
  V3D p_lidar(pi->x, pi->y, pi->z);
  V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_lidar +
                                  state_point.offset_T_L_I) +
               state_point.pos);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->r = pi->r;
  po->g = pi->g;
  po->b = pi->b;
  po->intensity = pi->intensity;
  po->offset_time = pi->offset_time;
  po->has_color = pi->has_color;
}

void LaserMappingNode::RGBpointLidarLidarToIMU(FastLioPoint const *const pi,
                                               FastLioPoint *const po) {
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar +
                 state_point.offset_T_L_I);
  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->r = pi->r;
  po->g = pi->g;
  po->b = pi->b;
  po->intensity = pi->intensity;
  po->offset_time = pi->offset_time;
  po->has_color = pi->has_color;
}

void LaserMappingNode::points_cache_collect() {
  PointVector points_history;
  // ikdtree.acquire_removed_points(points_history);
}

void LaserMappingNode::lasermap_fov_segment() {
  cub_needrm.clear();
  kdtree_delete_counter = 0;
  kdtree_delete_time = 0.0;
  pointLidarToWorld(XAxisPoint_body, XAxisPoint_world);
  V3D pos_LiD = pos_lid;
  if (!Localmap_Initialized) {
    for (int i = 0; i < 3; i++) {
      LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
      LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
    }
    Localmap_Initialized = true;
    return;
  }
  float dist_to_map_edge[3][2];
  bool need_move = false;
  for (int i = 0; i < 3; i++) {
    dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
    dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
    if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE ||
        dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
      need_move = true;
  }
  if (!need_move) return;
  BoxPointType New_LocalMap_Points, tmp_boxpoints;
  New_LocalMap_Points = LocalMap_Points;
  float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9,
                       double(DET_RANGE * (MOV_THRESHOLD - 1)));
  for (int i = 0; i < 3; i++) {
    tmp_boxpoints = LocalMap_Points;
    if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
      New_LocalMap_Points.vertex_max[i] -= mov_dist;
      New_LocalMap_Points.vertex_min[i] -= mov_dist;
      tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
      cub_needrm.push_back(tmp_boxpoints);
    } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
      New_LocalMap_Points.vertex_max[i] += mov_dist;
      New_LocalMap_Points.vertex_min[i] += mov_dist;
      tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
      cub_needrm.push_back(tmp_boxpoints);
    }
  }
  LocalMap_Points = New_LocalMap_Points;

  points_cache_collect();
  double delete_begin = omp_get_wtime();
  if (cub_needrm.size() > 0) {
    // kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
  }
  kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void LaserMappingNode::standard_pcl_cbk(
    const sensor_msgs::msg::PointCloud2::UniquePtr msg) {
  mtx_buffer.lock();
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
  s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LaserMappingNode::livox_pcl_cbk(
    const livox_ros_driver2::msg::CustomMsg::UniquePtr msg) {
  mtx_buffer.lock();
  double cur_time = get_time_sec(msg->header.stamp);
  double preprocess_start_time = omp_get_wtime();
  scan_count++;
  if (!is_first_lidar && cur_time < last_timestamp_lidar) {
    std::cerr << "lidar loop back, clear buffer" << std::endl;
    lidar_buffer.clear();
  }
  if (is_first_lidar) {
    is_first_lidar = false;
  }
  last_timestamp_lidar = cur_time;

  if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 &&
      !imu_buffer.empty() && !lidar_buffer.empty()) {
    printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",
           last_timestamp_imu, last_timestamp_lidar);
  }

  if (time_sync_en && !timediff_set_flg &&
      abs(last_timestamp_lidar - last_timestamp_imu) > 1 &&
      !imu_buffer.empty()) {
    timediff_set_flg = true;
    timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
    printf("Self sync IMU and LiDAR, time diff is %.10lf \n",
           timediff_lidar_wrt_imu);
  }

  FastLioPointCloud::Ptr ptr(new FastLioPointCloud());
  p_pre->process(msg, ptr);
  lidar_buffer.push_back(ptr);
  time_buffer.push_back(last_timestamp_lidar);

  s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LaserMappingNode::imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in) {
  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

  msg->header.stamp =
      get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu);
  if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en) {
    msg->header.stamp = rclcpp::Time(timediff_lidar_wrt_imu +
                                     get_time_sec(msg_in->header.stamp));
  }

  double timestamp = get_time_sec(msg->header.stamp);

  mtx_buffer.lock();

  if (timestamp < last_timestamp_imu) {
    std::cerr << "lidar loop back, clear buffer" << std::endl;
    imu_buffer.clear();
  }

  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

bool LaserMappingNode::sync_packages(MeasureGroup &meas,
                                     CamProcessVec &p_cams) {
  if (lidar_buffer.empty() || imu_buffer.empty()) {
    return false;
  }

  for (auto &p_cam : p_cams) {
    if (p_cam->img_buffer_.empty()) {
      return false;
    }
  }

  for (auto &p_cam : p_cams) {
    if (rclcpp::Time(p_cam->img_buffer_.back()->header.stamp).seconds() <
        time_buffer.front()) {
      return false;
    }
  }

  /*** push a lidar scan ***/
  if (!lidar_pushed) {
    meas.lidar = lidar_buffer.front();
    meas.lidar_beg_time = time_buffer.front();
    if (meas.lidar->points.size() <= 1)  // time too little
    {
      lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
      std::cerr << "Too few input point cloud!\n";
    } else if (meas.lidar->points.back().offset_time / double(1000) <
               0.5 * lidar_mean_scantime) {
      lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
    } else {
      scan_num++;
      lidar_end_time = meas.lidar_beg_time +
                       meas.lidar->points.back().offset_time / double(1000);
      lidar_mean_scantime +=
          (meas.lidar->points.back().offset_time / double(1000) -
           lidar_mean_scantime) /
          scan_num;
    }

    meas.lidar_end_time = lidar_end_time;

    lidar_pushed = true;
  }

  if (last_timestamp_imu < lidar_end_time) {
    return false;
  }

  /*** push imu data, and pop from imu buffer ***/
  double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
  meas.imu.clear();
  while ((!imu_buffer.empty()) && imu_time < lidar_end_time) {
    imu_time = get_time_sec(imu_buffer.front()->header.stamp);
    if (imu_time > lidar_end_time) break;
    meas.imu.push_back(imu_buffer.front());
    imu_buffer.pop_front();
  }

  lidar_buffer.pop_front();
  time_buffer.pop_front();
  lidar_pushed = false;
  return true;
}

bool LaserMappingNode::tensor_density_expection(Eigen::Vector3f &eig_val) {
  bool k1, k2, k3;

  k1 = eig_val(2) < tensor_d1;
  k1 |= eig_val(1) < tensor_d1;
  k1 |= eig_val(0) < 0.5 * tensor_d1;

  k2 = eig_val(2) < tensor_d2;
  k2 |= eig_val(1) < 0.75 * tensor_d2;
  k2 |= eig_val(0) < 0.75 * tensor_d2;

  k3 = eig_val(2) < (5.0 / 6.0) * tensor_d3;
  k3 |= eig_val(1) < (5.0 / 6.0) * tensor_d3;
  k3 |= eig_val(0) < (5.0 / 6.0) * tensor_d3;

  return k1 && k2 && k3;
}

void LaserMappingNode::compute_tensor_vote(int i, int j, Eigen::Matrix3f &A_j,
                                           bool first_pass) {
  Eigen::Vector3f p_i = map_cloud->points[i].getVector3fMap();
  Eigen::Vector3f p_j = map_cloud->points[j].getVector3fMap();
  float d_ij = (p_i - p_j).norm();
  float c_ij = std::exp(-std::pow(d_ij, 2) / filter_size_corner_min);
  Eigen::Vector3f r_ij = (p_i - p_j).normalized();
  Eigen::Matrix3f rrt = r_ij * r_ij.transpose();
  Eigen::Matrix3f R_ij = Eigen::Matrix3f::Identity() - 2.0 * rrt;
  Eigen::Matrix3f Rp_ij = (Eigen::Matrix3f::Identity() - 0.5 * rrt) * R_ij;
  Eigen::Matrix3f K_j = Eigen::Matrix3f::Identity();
  if (!first_pass) K_j = tensors_p2[j];
  A_j = c_ij * R_ij * K_j * Rp_ij;
}

void LaserMappingNode::compute_tensor_eigen(int i, Eigen::Matrix3f &tensor,
                                            bool first_pass) {
  Eigen::Vector3f eig_val, sali_val;
  Eigen::Matrix3f eig_vec, tensor_i2;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig_solver;

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
    // filters[i][1] = tensor_density_expection(eig_val);
  } else {
    sali_val(0) = eig_val(2) - eig_val(1);
    sali_val(1) = eig_val(1) - eig_val(0);
    sali_val(2) = eig_val(0);
    sali_val.maxCoeff(&saliency_idxs[i]);

    filters[i][2] = true;
    salivalues[i] = sali_val;
    eigenvalues[i] = (1.0 / (eig_val.array() + 1e-3)).matrix().normalized();
    eigenvalues[i] *= filter_size_corner_min;
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

void LaserMappingNode::tensor_vote_pass_1(int old_map_size,
                                          std::vector<int> &added_idxs,
                                          std::vector<int> &updated_idxs) {
  std::atomic<int> upd_idx = 0, new_neighbours_idx = 0;

#pragma omp parallel for
  for (int i = 0; i < added_idxs.size(); i++) {
    int map_i, loop_cnt;
    Eigen::MatrixXf K;
    std::vector<int> N_idxs;
    Eigen::Matrix3f tensor_i1;

    map_i = added_idxs[i];
    map_cloud->points[map_i].intensity = 0;
    map_cloud->points[map_i].curvature = 0;
    map_cloud->points[map_i].getNormalVector3fMap() = Eigen::Vector3f::Zero();

    ioctree.radiusNeighbors(map_cloud->points[map_i], map_search_radius,
                            N_idxs);
    neighbours[map_i] = N_idxs;

    loop_cnt = min(int(neighbours[map_i].size()), MAX_NEIGHBOURS);
    K = Eigen::MatrixXf::Zero(loop_cnt, 9);

#pragma omp parallel for
    for (int j = 0; j < loop_cnt; j++) {
      Eigen::Matrix3f A_j;
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
    Eigen::Matrix3f tensor_i1;
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
      Eigen::Matrix3f A_j;
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

void LaserMappingNode::tensor_vote_pass_2(std::vector<int> &added_idxs,
                                          std::vector<int> &updated_idxs) {
  Eigen::MatrixXf sali_vals;
  Eigen::VectorXi sali_filter;
  Eigen::Vector3f cur_mean_sali;

  int total_size = added_idxs.size() + updated_idxs.size();

  sali_vals = Eigen::MatrixXf::Zero(total_size, 3);
  sali_filter = Eigen::VectorXi::Zero(total_size);

#pragma omp parallel for
  for (int i = 0; i < total_size; i++) {
    Eigen::MatrixXf K;
    Eigen::VectorXi K_filter;
    Eigen::Matrix3f tensor_i2;
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

      Eigen::Matrix3f A_j;
      compute_tensor_vote(map_i, map_j, A_j, false);
      K.row(j) = A_j.reshaped(1, 9);
      K_filter(j) = 1;
    }

    filter_cnt = K_filter.sum();
    if (filter_cnt < NUM_MATCH_POINTS) continue;

    tensor_i2 = K.colwise().sum().reshaped(3, 3);
    tensor_i2 /= float(filter_cnt);
    compute_tensor_eigen(map_i, tensor_i2, false);

    sali_filter(i) = filters[map_i][2];
    sali_vals.row(i) = salivalues[map_i];
  }

  cur_mean_sali = sali_vals.colwise().sum() / float(sali_filter.sum());
  mean_sali = (cur_mean_sali + (float(map_counter) * mean_sali)) /
              (float(map_counter) + 1);
}

void LaserMappingNode::map_incremental(bool init_map) {
  std::vector<int> added_idxs, new_idxs, updated_idxs;

  if (init_map) {
    feats_down_world->resize(feats_undistort->size());
#pragma omp parallel for
    for (int i = 0; i < feats_undistort->size(); i++) {
      pointLidarToWorld(&(feats_undistort->points[i]),
                        &(feats_down_world->points[i]));
    }
  } else {
    feats_down_world->resize(feats_down_body->size());
#pragma omp parallel for
    for (int i = 0; i < feats_down_body->size(); i++) {
      pointLidarToWorld(&(feats_down_body->points[i]),
                        &(feats_down_world->points[i]));
    }
  }

  double st_time = omp_get_wtime();
  int old_map_size = map_cloud->size();

  ioctree.set_bucket_size(map_bucket_size);
  ioctree.update(*feats_down_world, added_idxs, new_idxs);
  *map_cloud += FastLioPointCloud(*feats_down_world, added_idxs);

  update_cnt.resize(map_cloud->size(), 0);
  update_idx.resize(map_cloud->size(), 0);
  updated_pt.resize(map_cloud->size(), 0);
  saliency_idxs.resize(map_cloud->size(), 0);
  neighbours.resize(map_cloud->size(), std::vector<int>());
  filters.resize(map_cloud->size(), std::vector<bool>(3, false));

  tensors_p1.resize(map_cloud->size(), Eigen::Matrix3f::Zero());
  tensors_p2.resize(map_cloud->size(), Eigen::Matrix3f::Zero());
  salivalues.resize(map_cloud->size(), Eigen::Vector3f::Zero());
  eigenvalues.resize(map_cloud->size(), Eigen::Vector3f::Zero());
  eigenvectors.resize(map_cloud->size(), Eigen::Matrix3f::Zero());

  std::cerr << "Map size: " << map_cloud->size() << std::endl;
  std::cerr << "ioctree size: " << ioctree.size() << std::endl;

  std::cerr << "Added idxs size: " << new_idxs.size() << std::endl;
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
  kdtree_incremental_time = omp_get_wtime() - st_time;
}

void LaserMappingNode::publish_frame_world() {
  FastLioPointCloud::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort
                                                        : feats_down_body);
  int size = laserCloudFullRes->points.size();
  FastLioPointCloud::Ptr laserCloudWorld(new FastLioPointCloud(size, 1));

  for (int i = 0; i < size; i++) {
    RGBpointLidarToWorld(&laserCloudFullRes->points[i],
                         &laserCloudWorld->points[i]);
  }

  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
  laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
  laserCloudmsg.header.frame_id = "odom_fastlio";
  pubLaserCloudFull_->publish(laserCloudmsg);
}

void LaserMappingNode::publish_frame_body() {
  int size = feats_undistort->points.size();
  FastLioPointCloud::Ptr laserCloudIMUBody(new FastLioPointCloud(size, 1));

  for (int i = 0; i < size; i++) {
    RGBpointLidarLidarToIMU(&feats_undistort->points[i],
                            &laserCloudIMUBody->points[i]);
  }

  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
  laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
  laserCloudmsg.header.frame_id = "imu_fastlio";
  pubLaserCloudFull_body_->publish(laserCloudmsg);
}

void LaserMappingNode::publish_effect_world() {
  FastLioPointCloud::Ptr laserCloudWorld(
      new FastLioPointCloud(effct_feat_num, 1));
  for (int i = 0; i < effct_feat_num; i++) {
    RGBpointLidarToWorld(&laserCloudOri->points[i],
                         &laserCloudWorld->points[i]);
  }
  sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
  laserCloudFullRes3.header.frame_id = "odom_fastlio";
  pubLaserCloudEffect_->publish(laserCloudFullRes3);
}

void LaserMappingNode::publish_markers() {
  std::atomic<int> marker_idx = 0;
  int start_idx, end_idx, step_idx, count_idx;
  visualization_msgs::msg::MarkerArray marker_array;

  start_idx = marker_start_idx;
  end_idx = map_cloud->points.size();
  step_idx = ceil(1e-2 * (end_idx - start_idx));
  count_idx = (end_idx - start_idx) / step_idx;

  marker_array.markers.resize(count_idx);
#pragma omp parallel for
  for (int i = 0; i < count_idx; i++) {
    Eigen::Quaternionf quat;
    visualization_msgs::msg::Marker marker;

    int map_idx = start_idx + (i * step_idx);
    if (!filters[map_idx][2]) continue;

    marker.id = map_idx;
    marker.frame_locked = true;
    marker.ns = "map_primitives";
    marker.lifetime = rclcpp::Duration(0, 0);
    marker.header.frame_id = "odom_fastlio";
    marker.header.stamp = get_ros_time(lidar_end_time);
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
  pubMarker_->publish(marker_array);
}

void LaserMappingNode::publish_map() {
  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  pcl::toROSMsg(*map_cloud, laserCloudmsg);
  laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
  laserCloudmsg.header.frame_id = "odom_fastlio";
  pubLaserCloudMap_->publish(laserCloudmsg);
}

void LaserMappingNode::save_to_pcd() {
  pcl::PCDWriter pcd_writer;
  pcd_writer.writeBinary(map_file_path, *pcl_wait_pub);
}

template <typename T>
void LaserMappingNode::set_posestamp(T &out) {
  out.pose.position.x = state_point.pos(0);
  out.pose.position.y = state_point.pos(1);
  out.pose.position.z = state_point.pos(2);
  out.pose.orientation.x = geoQuat.x;
  out.pose.orientation.y = geoQuat.y;
  out.pose.orientation.z = geoQuat.z;
  out.pose.orientation.w = geoQuat.w;
}

void LaserMappingNode::publish_odometry() {
  odomAftMapped.header.frame_id = "odom_fastlio";
  odomAftMapped.child_frame_id = "imu_fastlio";
  odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
  set_posestamp(odomAftMapped.pose);
  auto P = kf.get_P();
  for (int i = 0; i < 6; i++) {
    int k = i < 3 ? i + 3 : i - 3;
    odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
    odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
    odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
    odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
    odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
    odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
  }
  pubOdomAftMapped_->publish(odomAftMapped);

  geometry_msgs::msg::TransformStamped trans;
  trans.header.frame_id = "odom_fastlio";
  trans.child_frame_id = "imu_fastlio";
  trans.header.stamp = get_ros_time(lidar_end_time);
  trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
  trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
  trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
  trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
  trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
  trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
  trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
  tf_br_->sendTransform(trans);
}

void LaserMappingNode::publish_path() {
  path.header.stamp = get_ros_time(lidar_end_time);
  path.header.frame_id = "odom_fastlio";

  set_posestamp(msg_body_pose);
  msg_body_pose.header.stamp =
      get_ros_time(lidar_end_time);  // ros::Time().fromSec(lidar_end_time);
  msg_body_pose.header.frame_id = "odom_fastlio";

  path.poses.push_back(msg_body_pose);
  pubPath_->publish(path);
}

void LaserMappingNode::h_share_model(
    state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
  double match_start = omp_get_wtime();
  laserCloudOri->clear();
  corr_normvect->clear();
  total_residual = 0.0;

#pragma omp parallel for
  for (int i = 0; i < feats_down_size; i++) {
    FastLioPoint &point_body = feats_down_body->points[i];
    FastLioPoint &point_world = feats_down_world->points[i];

    /* transform to world frame */
    V3D p_lidar(point_body.x, point_body.y, point_body.z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_lidar + s.offset_T_L_I) + s.pos);
    point_world.x = p_global(0);
    point_world.y = p_global(1);
    point_world.z = p_global(2);
    point_world.intensity = point_body.intensity;
    point_world.r = point_body.r;
    point_world.g = point_body.g;
    point_world.b = point_body.b;
    point_world.has_color = point_body.has_color;

    vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

    auto &points_near = Nearest_Points[i];

    if (ekfom_data.converge) {
      /** Find the closest surfaces in the map **/
      // ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near,
      // pointSearchSqDis);
      ioctree.knnNeighbors(point_world, NUM_MATCH_POINTS, points_near,
                           pointSearchSqDis);
      point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false
                               : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5
                                   ? false
                                   : true;
    }

    if (!point_selected_surf[i]) continue;

    VF(4) pabcd;
    point_selected_surf[i] = false;
    if (esti_plane(pabcd, points_near, 0.1f)) {
      float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y +
                  pabcd(2) * point_world.z + pabcd(3);
      float s = 1 - 0.9 * fabs(pd2) / sqrt(p_lidar.norm());

      if (s > 0.9) {
        point_selected_surf[i] = true;
        normvec->points[i].x = pabcd(0);
        normvec->points[i].y = pabcd(1);
        normvec->points[i].z = pabcd(2);
        normvec->points[i].intensity = pd2;
        res_last[i] = abs(pd2);
      }
    }
  }

  effct_feat_num = 0;

  for (int i = 0; i < feats_down_size; i++) {
    if (point_selected_surf[i]) {
      laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
      corr_normvect->points[effct_feat_num] = normvec->points[i];
      total_residual += res_last[i];
      effct_feat_num++;
    }
  }

  std::cerr << "Effective points: " << effct_feat_num << std::endl;

  if (effct_feat_num < 1) {
    ekfom_data.valid = false;
    std::cerr << "No Effective Points!" << std::endl;
    // ROS_WARN("No Effective Points! \n");
    return;
  }

  std::cerr << "Mean Residual: " << total_residual / effct_feat_num
            << std::endl;
  res_mean_last = total_residual / effct_feat_num;
  match_time += omp_get_wtime() - match_start;
  double solve_start_ = omp_get_wtime();

  /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
  ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);  // 23
  ekfom_data.h.resize(effct_feat_num);

  for (int i = 0; i < effct_feat_num; i++) {
    const FastLioPoint &laser_p = laserCloudOri->points[i];
    V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
    M3D point_be_crossmat;
    point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
    V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);

    /*** get the normal vector of closest surface/corner ***/
    const FastLioPoint &norm_p = corr_normvect->points[i];
    V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

    /*** calculate the Measuremnt Jacobian matrix H ***/
    V3D C(s.rot.conjugate() * norm_vec);
    V3D A(point_crossmat * C);

    double res = norm_p.intensity;

    if (extrinsic_est_en) {
      V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() *
            C);  // s.rot.conjugate()*norm_vec);
      ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec(0), norm_vec(1),
          norm_vec(2), VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
    } else {
      ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec(0), norm_vec(1),
          norm_vec(2), VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    }

    /*** Measuremnt: distance to the closest surface/corner ***/
    ekfom_data.h(i) = -res;
  }
  solve_time += omp_get_wtime() - solve_start_;
}

void LaserMappingNode::compute_geometric_primitive(int map_i, int sali_idx,
                                                   Eigen::Vector3f &p_world,
                                                   Eigen::Vector3f &norm_vec) {
  Eigen::Vector3f q, q_dash, p_dash, n_world;

  n_world = map_cloud->points[map_i].getVector3fMap();

  if (sali_idx == 0) {
    // Point to plane
    q = p_world - n_world;
    q_dash = q.dot(eigenvectors[map_i].col(2)) * eigenvectors[map_i].col(2);
    p_dash = p_world - q_dash;
    norm_vec = p_world - p_dash;
  } else if (sali_idx == 1) {
    //  Point to curve
    q = p_world - n_world;
    q_dash = q.dot(eigenvectors[map_i].col(0)) * eigenvectors[map_i].col(0);
    p_dash = n_world + q_dash;
    norm_vec = p_world - p_dash;
  } else if (sali_idx == 2) {
    //  Point to junction
    norm_vec = p_world - n_world;
  }
}

void LaserMappingNode::tensor_registration(
    state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
  Eigen::MatrixXd h(feats_down_size, 1);
  Eigen::MatrixXd h_x(feats_down_size, 12);
  std::atomic<int> feat_cnt = 0, plane_cnt = 0, curve_cnt = 0, junct_cnt = 0;

  total_residual = 0.0;

  double match_start = omp_get_wtime();
  double solve_start_ = omp_get_wtime();

#pragma omp parallel for
  for (int i = 0; i < feats_down_size; i++) {
    int sali_idx, map_i;
    float residual;
    Eigen::Vector3f c, a;
    Eigen::Matrix3f P_skew;
    std::vector<int> N_idxs, N_p_idxs;
    std::vector<float> N_dst, N_p_dst;
    Eigen::Vector3f sali, p_world, n_world, p_dash, q, q_dash, norm_vec,
        eig_vals;

    FastLioPoint point_imu, point_proj;
    FastLioPoint &point_lidar = feats_down_body->points[i];
    FastLioPoint &point_world = feats_down_world->points[i];

    pointLidarToIMU_ikfom(&point_lidar, &point_imu, s);
    pointLidarToWorld_ikfom(&point_lidar, &point_world, s);

    ioctree.knnNeighbors(point_world, 1, N_idxs, N_dst);
    map_i = N_idxs[0];
    if (sqrt(N_dst[0]) > map_search_radius || !filters[map_i][2]) continue;

    sali_idx = saliency_idxs[map_i];
    if (salivalues[map_i](sali_idx) < mean_sali(sali_idx)) continue;

    p_world = point_world.getVector3fMap();
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

    P_skew << SKEW_SYM_MATRX(point_imu.getVector3fMap());

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
  solve_time += omp_get_wtime() - solve_start_;
}

void LaserMappingNode::compute_eigendecomposition(
    state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
  int avg_num_neighbours = 0;
  std::atomic_int feat_cnt = 0, line_cnt = 0, plane_cnt = 0, ellipse_cnt = 0;
  Eigen::MatrixXd h(feats_down_size, 1);
  Eigen::MatrixXd h_x(feats_down_size, 12);

  total_residual = 0.0;

  double match_start = omp_get_wtime();
  double solve_start_ = omp_get_wtime();

  // #pragma omp parallel for
  for (int i = 0; i < feats_down_size; i++) {
    int prim;
    float res;
    Eigen::VectorXf N_norm;
    Eigen::MatrixXf N, N_bar;
    std::vector<float> N_dist;
    PointVector near_pt;
    Eigen::Matrix3f Phi, P_skew;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig;
    Eigen::Matrix3f Cov2, Cov = Eigen::Matrix3f::Zero();
    Eigen::Vector3f N_mean, Lambda, saliency, p_lidar, p_imu, p_world, p_dash,
        q, q_dash, norm_vec, C, A;

    FastLioPoint &point_body = feats_down_body->points[i];
    FastLioPoint &point_world = feats_down_world->points[i];

    p_lidar = point_body.getVector3fMap();
    p_imu =
        s.offset_R_L_I.cast<float>() * p_lidar + s.offset_T_L_I.cast<float>();
    p_world = s.rot.cast<float>() * p_imu + s.pos.cast<float>();

    point_world = point_body;
    point_world.getVector3fMap() = p_world;

    // ioctree.knnNeighbors(point_world, 1, near_pt, N_dist);
    // if (N_dist[0] > filter_size_corner_min) {
    //   // std::cerr << "Not enough neighbours!" << std::endl;
    //   continue;
    // }
    // ioctree.radiusNeighbors(near_pt[0], filter_size_corner_min, N, N_dist);

    // std::cerr << "world_pt: " << point_world.getVector3fMap() << std::endl;
    // std::cerr << "near_pt: " << near_pt[0].getVector3fMap() << std::endl;

    // if (N.rows() < NUM_MATCH_POINTS) {
    //  std::cerr << "Not enough neighbours!" << std::endl;
    // continue;
    //}
    // std::cerr << "Neighbours: " << N.rows() << std::endl;

    N = Eigen::MatrixXf::Zero(1000, 3);
    N.col(0) = filter_size_corner_min * (Eigen::VectorXf::Random(1000));
    N.col(1) = filter_size_corner_min * (Eigen::VectorXf::Random(1000));
    // N.col(2) = filter_size_corner_min * (Eigen::VectorXf::Random(1000));

    N_mean = Eigen::Vector3f(1, 1, 0);
    N_bar = (N.rowwise() - N_mean.transpose());
    N_norm = N_bar.rowwise().norm();
    N_bar.rowwise().normalize();

    // #pragma omp parallel for
    for (int j = 0; j < N.rows(); j++) {
      if (N_norm(j) == 0) {
        continue;
      }
      Eigen::Matrix3f N_j = N_bar.row(j).transpose() * N_bar.row(j);
      Eigen::Matrix3f R_ij = Eigen::Matrix3f::Identity() - 2.0 * N_j;
      Eigen::Matrix3f Rt_ij = (Eigen::Matrix3f::Identity() - 0.5 * N_j) * R_ij;
      float c_ij = std::exp(-std::pow(N_norm(j), 2) / filter_size_corner_min);
      Cov += c_ij * R_ij * Eigen::Matrix3f::Identity() * Rt_ij;
    }

    eig.computeDirect(Cov);
    Phi = eig.eigenvectors();
    Lambda = eig.eigenvalues().cwiseAbs();
    std::cerr << "Eigenvalues TV: " << Lambda.transpose() << std::endl;

    Cov2 = ((Lambda(2) - Lambda(1)) / N.rows()) * Phi.col(2) *
               Phi.col(2).transpose() +
           ((Lambda(1) - Lambda(0)) / N.rows()) *
               (Phi.col(2) * Phi.col(2).transpose() +
                Phi.col(1) * Phi.col(1).transpose());

    Cov = Eigen::Matrix3f::Zero();
    for (int j = 0; j < N.rows(); j++) {
      if (N_norm(j) == 0) {
        continue;
      }
      Eigen::Matrix3f N_j = N_bar.row(j).transpose() * N_bar.row(j);
      Eigen::Matrix3f R_ij = Eigen::Matrix3f::Identity() - 2.0 * N_j;
      Eigen::Matrix3f Rt_ij = (Eigen::Matrix3f::Identity() - 0.5 * N_j) * R_ij;
      float c_ij = std::exp(-std::pow(N_norm(j), 2) / filter_size_corner_min);
      Cov += c_ij * R_ij * Cov2 * Rt_ij;
    }

    eig.computeDirect(Cov);
    Phi = eig.eigenvectors();
    Lambda = eig.eigenvalues().cwiseAbs();
    std::cerr << "Eigenvalues TV2: " << Lambda.transpose() << std::endl;
    std::cerr << "Eigenvector Max: " << Phi.col(2) << std::endl;
    std::cerr << "Eigenvector Mean: " << Phi.col(1) << std::endl;
    std::cerr << "Eigenvector Min: " << Phi.col(0) << std::endl;
    std::cerr << "Mean vec: "
              << (N.colwise().mean().transpose() - N_mean).normalized()
              << std::endl;
    std::cerr << "Mean vec norm: "
              << (N.colwise().mean().transpose() - N_mean).norm() << std::endl;

    // N_mean = N.colwise().mean();
    // N_bar = (N.rowwise() - N_mean.transpose());
    // Cov = (N_bar.adjoint() * N_bar) / float(N_bar.rows() - 1);
    // eig.compute(Cov);
    // Phi = eig.eigenvectors();
    // Lambda = eig.eigenvalues().cwiseAbs();
    // std::cerr << "Eigenvalues PCA: " << Lambda.transpose() << std::endl;

    if (eig.info() != Eigen::Success) {
      std::cerr << "Eigendecomposition failed!" << std::endl;
      continue;
    }

    if (eig.eigenvalues().minCoeff() < 0.0) {
      std::cerr << "Negative eigenvalue!" << std::endl;
      continue;
    }

    avg_num_neighbours += N.rows();

    std::cerr << "N: " << N.rows() << std::endl;

    saliency << Lambda(2) - Lambda(1), Lambda(1) - Lambda(0), Lambda(0);
    saliency.maxCoeff(&prim);
    Lambda = Lambda.cwiseSqrt();

    std::cerr << "Saliency: " << saliency << std::endl;

    if (prim == 0) {
      // Point to plane
      q = p_world - N_mean;
      q_dash = q.dot(Phi.col(2)) * Phi.col(2);
      p_dash = p_world - q_dash;
      norm_vec = p_world - p_dash;
      ++line_cnt;
      std::cerr << Phi.col(2) << std::endl;
    } else if (prim == 1) {
      // Point to curve
      q = p_world - N_mean;
      q_dash = q.dot(Phi.col(0)) * Phi.col(0);
      p_dash = p_world - q_dash;
      norm_vec = p_world - p_dash;
      ++plane_cnt;
      std::cerr << Phi.col(0) << std::endl;
    } else if (prim == 2) {
      // Point to ellipsoid
      p_dash = Phi.transpose() * (p_world - N_mean);
      if (!projectEllipsoid(q_dash.data(), p_dash.data(), Lambda.data())) {
        continue;
      }
      q = Phi * q_dash + N_mean;
      norm_vec = p_world - q;
      ++ellipse_cnt;
      std::cerr << Phi.col(1) << std::endl;
    }

    res = norm_vec.norm();
    norm_vec.normalize();

    P_skew << SKEW_SYM_MATRX(p_imu);

    C = s.rot.conjugate().cast<float>() * norm_vec;
    A = P_skew * C;

    int feat_num = ++feat_cnt;
    // std::cerr << "Feat num: " << feat_num << std::endl;
    h_x.row(feat_num - 1) << norm_vec(0), norm_vec(1), norm_vec(2),
        VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    h(feat_num - 1) = -res;

    total_residual += res;
  }

  h.conservativeResize(feat_cnt, 1);
  h_x.conservativeResize(feat_cnt, 12);
  ekfom_data.h = h;
  ekfom_data.h_x = h_x;

  res_mean_last = total_residual / feat_cnt;
  avg_num_neighbours /= feats_down_size;

  std::cerr << "Res mean: " << res_mean_last << std::endl;
  std::cerr << "Num feats: " << feat_cnt << std::endl;
  std::cerr << "Num planes: " << line_cnt << std::endl;
  std::cerr << "Num curves: " << plane_cnt << std::endl;
  std::cerr << "Num junctions: " << ellipse_cnt << std::endl;
  std::cerr << "Average number of neighbours: " << avg_num_neighbours
            << std::endl;

  match_time += omp_get_wtime() - match_start;
  solve_time += omp_get_wtime() - solve_start_;
}

LaserMappingNode::LaserMappingNode(
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node("laser_mapping", options),
      map_cloud(new FastLioPointCloud()),
      featsFromMap(new FastLioPointCloud()),
      feats_undistort(new FastLioPointCloud()),
      feats_undistort_world(new FastLioPointCloud()),
      feats_down_body(new FastLioPointCloud()),
      feats_down_world(new FastLioPointCloud()),
      normvec(new FastLioPointCloud(100000, 1)),
      laserCloudOri(new FastLioPointCloud(100000, 1)),
      corr_normvect(new FastLioPointCloud(100000, 1)),
      pcl_wait_pub(new FastLioPointCloud()),
      pcl_wait_save(new FastLioPointCloud()),
      extrinT(3, 0.0),
      extrinR(9, 0.0),
      XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0),
      XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0),
      position_last(Zero3d),
      Lidar_T_wrt_IMU(Zero3d),
      Lidar_R_wrt_IMU(Eye3d),
      p_pre(new Preprocess()),
      p_imu(new ImuProcess()) {
  this->declare_parameter<int>("publish.pub_map_n_secs", 1);
  this->declare_parameter<bool>("publish.path_en", true);
  this->declare_parameter<bool>("publish.effect_map_en", false);
  this->declare_parameter<bool>("publish.map_en", false);
  this->declare_parameter<bool>("publish.scan_publish_en", true);
  this->declare_parameter<bool>("publish.dense_publish_en", true);
  this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
  this->declare_parameter<int>("max_iteration", 4);
  this->declare_parameter<string>("map_file_path", "");
  this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
  this->declare_parameter<string>("common.imu_topic", "/livox/imu");
  this->declare_parameter<bool>("common.time_sync_en", false);
  this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
  this->declare_parameter<double>("filter_size_corner", 0.5);
  this->declare_parameter<double>("filter_size_surf", 0.5);
  this->declare_parameter<double>("filter_size_map", 0.5);
  this->declare_parameter<double>("cube_side_length", 200.);
  this->declare_parameter<float>("mapping.det_range", 300.);
  this->declare_parameter<double>("mapping.fov_degree", 180.);
  this->declare_parameter<double>("mapping.gyr_cov", 0.1);
  this->declare_parameter<double>("mapping.acc_cov", 0.1);
  this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
  this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
  this->declare_parameter<double>("preprocess.blind", 0.01);
  this->declare_parameter<int>("preprocess.lidar_type", AVIA);
  this->declare_parameter<int>("preprocess.scan_line", 16);
  this->declare_parameter<int>("preprocess.timestamp_unit", US);
  this->declare_parameter<int>("preprocess.scan_rate", 10);
  this->declare_parameter<int>("point_filter_num", 2);
  this->declare_parameter<bool>("feature_extract_enable", false);
  this->declare_parameter<bool>("runtime_pos_log_enable", false);
  this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
  this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
  this->declare_parameter<int>("pcd_save.interval", -1);
  this->declare_parameter<vector<double>>("mapping.extrinsic_T",
                                          vector<double>());
  this->declare_parameter<vector<double>>("mapping.extrinsic_R",
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
  this->get_parameter_or<bool>("publish.path_en", path_en, true);
  this->get_parameter_or<bool>("publish.effect_map_en", effect_pub_en, false);
  this->get_parameter_or<bool>("publish.map_en", map_pub_en, false);
  this->get_parameter_or<bool>("publish.scan_publish_en", scan_pub_en, true);
  this->get_parameter_or<bool>("publish.dense_publish_en", dense_pub_en, true);
  this->get_parameter_or<bool>("publish.scan_bodyframe_pub_en",
                               scan_body_pub_en, true);
  this->get_parameter_or<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
  this->get_parameter_or<string>("map_file_path", map_file_path, "");
  this->get_parameter_or<string>("common.lid_topic", lid_topic, "/livox/lidar");
  this->get_parameter_or<string>("common.imu_topic", imu_topic, "/livox/imu");
  this->get_parameter_or<bool>("common.time_sync_en", time_sync_en, false);
  this->get_parameter_or<double>("common.time_offset_lidar_to_imu",
                                 time_diff_lidar_to_imu, 0.0);
  this->get_parameter_or<double>("filter_size_corner", filter_size_corner_min,
                                 0.5);
  this->get_parameter_or<double>("filter_size_surf", filter_size_surf_min, 0.5);
  this->get_parameter_or<double>("filter_size_map", filter_size_map_min, 0.5);
  this->get_parameter_or<double>("cube_side_length", cube_len, 200.f);
  this->get_parameter_or<float>("mapping.det_range", DET_RANGE, 300.f);
  this->get_parameter_or<double>("mapping.fov_degree", fov_deg, 180.f);
  this->get_parameter_or<double>("mapping.gyr_cov", gyr_cov, 0.1);
  this->get_parameter_or<double>("mapping.acc_cov", acc_cov, 0.1);
  this->get_parameter_or<double>("mapping.b_gyr_cov", b_gyr_cov, 0.0001);
  this->get_parameter_or<double>("mapping.b_acc_cov", b_acc_cov, 0.0001);
  this->get_parameter_or<double>("preprocess.blind", p_pre->blind, 0.01);
  this->get_parameter_or<int>("preprocess.lidar_type", p_pre->lidar_type, AVIA);
  this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 16);
  this->get_parameter_or<int>("preprocess.timestamp_unit", p_pre->time_unit,
                              US);
  this->get_parameter_or<int>("preprocess.scan_rate", p_pre->SCAN_RATE, 10);
  this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num, 2);
  this->get_parameter_or<bool>("feature_extract_enable", p_pre->feature_enabled,
                               false);
  this->get_parameter_or<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
  this->get_parameter_or<bool>("mapping.extrinsic_est_en", extrinsic_est_en,
                               true);
  this->get_parameter_or<bool>("pcd_save.pcd_save_en", pcd_save_en, false);
  this->get_parameter_or<int>("pcd_save.interval", pcd_save_interval, -1);
  this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT,
                                         vector<double>());
  this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR,
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

  ioctree.set_min_extent(filter_size_map_min);
  ioctree.set_bucket_size(1);
  ioctree_scan.set_min_extent(filter_size_surf_min);
  ioctree_scan.set_bucket_size(1);

  mean_sali = Eigen::Vector3f::Zero();

  new_neighbours_map_idx = std::vector<int>(100000);
  new_neighbours_size = std::vector<std::atomic<int>>(100000);
  new_neighbours = std::vector<std::vector<int>>(100000, std::vector<int>(100));

  tensor_sigma = filter_size_map_min;
  tensor_radius = filter_size_corner_min;

  tensor_d1 = (std::sqrt(M_PI * tensor_sigma) *
               std::erf(tensor_radius / std::sqrt(tensor_sigma))) /
              (2. * tensor_radius);
  tensor_d2 =
      (tensor_sigma -
       tensor_sigma * std::exp(-tensor_radius * tensor_radius / tensor_sigma)) /
      (tensor_radius * tensor_radius);
  tensor_d3 = 3. * tensor_sigma *
              (std::sqrt(M_PI * tensor_sigma) *
                   std::erf(tensor_radius / std::sqrt(tensor_sigma)) -
               2. * tensor_radius *
                   std::exp(-tensor_radius * tensor_radius / tensor_sigma)) /
              (4. * tensor_radius * tensor_radius * tensor_radius);

  p_pre->blind_sqr = p_pre->blind * p_pre->blind;

  RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d", p_pre->lidar_type);

  FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
  HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

  _featsArray.reset(new FastLioPointCloud());

  memset(point_selected_surf, true, sizeof(point_selected_surf));
  memset(res_last, -1000.0f, sizeof(res_last));

  Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
  Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);

  p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
  p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
  p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

  fill(epsi, epsi + 23, 0.001);
  kf.init_dyn_share(get_f, df_dx, df_dw,
                    std::bind(&LaserMappingNode::tensor_registration, this,
                              std::placeholders::_1, std::placeholders::_2),
                    NUM_MAX_ITERATIONS, epsi);

  /*** debug record ***/
  // FILE *fp;
  string pos_log_dir = root_dir + "/Log/pos_log.txt";
  fp = fopen(pos_log_dir.c_str(), "w");

  // ofstream fout_pre, fout_out, fout_dbg;
  fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), ios::out);
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
  fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"), ios::out);
  if (fout_pre && fout_out)
    cout << "~~~~" << ROOT_DIR << " file opened" << endl;
  else
    cout << "~~~~" << ROOT_DIR << " doesn't exist" << endl;

  /*** ROS subscribe initialization ***/
  if (p_pre->lidar_type == AVIA) {
    sub_pcl_livox_ =
        this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            lid_topic, rclcpp::SensorDataQoS(),
            std::bind(&LaserMappingNode::livox_pcl_cbk, this,
                      std::placeholders::_1));
  } else {
    sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lid_topic, rclcpp::SensorDataQoS(),
        std::bind(&LaserMappingNode::standard_pcl_cbk, this,
                  std::placeholders::_1));
  }
  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&LaserMappingNode::imu_cbk, this, std::placeholders::_1));

  pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered", 1);
  pubLaserCloudMap_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 1);
  pubOdomAftMapped_ =
      this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 1);
  pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 1);
  pubMarker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/visualization_marker", 1);
  tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  loop_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pub_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  loop_timer_ = rclcpp::create_timer(
      this, this->get_clock(), std::chrono::milliseconds(10),
      std::bind(&LaserMappingNode::timer_callback, this));
  // pub_odom_timer_ =
  //     rclcpp::create_timer(this, this->get_clock(),
  //                          std::chrono::milliseconds(1000 /
  //                          p_pre->SCAN_RATE),
  //                          std::bind(&LaserMappingNode::publish_odometry,
  //                          this), pub_callback_group_);
  pub_path_timer_ = rclcpp::create_timer(
      this, this->get_clock(),
      std::chrono::milliseconds(1000 / p_pre->SCAN_RATE),
      std::bind(&LaserMappingNode::publish_path, this), pub_callback_group_);
  pub_scan_timer_ = rclcpp::create_timer(
      this, this->get_clock(),
      std::chrono::milliseconds(1000 / p_pre->SCAN_RATE),
      std::bind(&LaserMappingNode::publish_frame_world, this),
      pub_callback_group_);
  pub_map_timer_ = rclcpp::create_timer(
      this, this->get_clock(), std::chrono::milliseconds(pub_map_n_secs * 1000),
      std::bind(&LaserMappingNode::publish_map, this), pub_callback_group_);
  pub_marker_timer_ = rclcpp::create_timer(
      this, this->get_clock(), std::chrono::milliseconds(pub_map_n_secs * 1000),
      std::bind(&LaserMappingNode::publish_markers, this), pub_callback_group_);

  map_save_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "map_save", std::bind(&LaserMappingNode::map_save_callback, this,
                            std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Node init finished.");
}

LaserMappingNode::~LaserMappingNode() {
  fout_out.close();
  fout_pre.close();
  fclose(fp);

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. pcd save will largely influence the real-time performences **/
  if (pcl_wait_save->size() > 0 && pcd_save_en) {
    string file_name = string("scans.pcd");
    string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
    pcl::PCDWriter pcd_writer;
    cout << "current scan saved to /PCD/" << file_name << endl;
    pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
  }

  if (runtime_pos_log) {
    vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;
    FILE *fp2;
    string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
    fp2 = fopen(log_dir.c_str(), "w");
    fprintf(fp2,
            "time_stamp, total time, scan point size, incremental time, "
            "search "
            "time, delete size, delete time, tree size st, tree size "
            "end, add "
            "point size, preprocess time\n");
    for (int i = 0; i < time_log_counter; i++) {
      fprintf(fp2, "%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",
              T1[i], s_plot[i], int(s_plot2[i]), s_plot3[i], s_plot4[i],
              int(s_plot5[i]), s_plot6[i], int(s_plot7[i]), int(s_plot8[i]),
              int(s_plot10[i]), s_plot11[i]);
      t.push_back(T1[i]);
      s_vec.push_back(s_plot9[i]);
      s_vec2.push_back(s_plot3[i] + s_plot6[i]);
      s_vec3.push_back(s_plot4[i]);
      s_vec5.push_back(s_plot[i]);
    }
    fclose(fp2);
  }
}

void LaserMappingNode::init_cam_process() {
  if (cam_init) {
    return;
  }
  cam_init = true;
  if (cam_topics.empty()) {
    RCLCPP_INFO(this->get_logger(), "No camera topics, skip camera process");
    return;
  }
  if (cam_topics.size() * 3 != T_cam_lidars.size()) {
    RCLCPP_ERROR(this->get_logger(),
                 "The number of camera topics and T_cam_lidars are not "
                 "consistent, skip camera process");
    return;
  }
  if (cam_topics.size() * 9 != R_cam_lidars.size()) {
    RCLCPP_ERROR(this->get_logger(),
                 "The number of camera topics and R_cam_lidars are not "
                 "consistent, skip camera process");
    return;
  }
  if (cam_topics.size() * 9 != cam_intrinsics.size()) {
    RCLCPP_ERROR(this->get_logger(),
                 "The number of camera topics and cam_intrinsics are not "
                 "consistent, skip camera process");
    return;
  }
  for (int i = 0; i < cam_topics.size(); i++) {
    p_cams.push_back(std::make_shared<CamProcess>(cam_frame_rate, cam_topics[i],
                                                  shared_from_this()));
    V3D Lidar_T_wrt_Cam(Zero3d);
    M3D Lidar_R_wrt_Cam(Eye3d);
    M3D cam_intrinsic_mat(Eye3d);
    vector<double> T_cam_lidar(T_cam_lidars.begin() + i * 3,
                               T_cam_lidars.begin() + i * 3 + 3);
    vector<double> R_cam_lidar(R_cam_lidars.begin() + i * 9,
                               R_cam_lidars.begin() + i * 9 + 9);
    vector<double> cam_intrinsic(cam_intrinsics.begin() + i * 9,
                                 cam_intrinsics.begin() + i * 9 + 9);
    Lidar_T_wrt_Cam << VEC_FROM_ARRAY(T_cam_lidar);
    Lidar_R_wrt_Cam << MAT_FROM_ARRAY(R_cam_lidar);
    cam_intrinsic_mat << MAT_FROM_ARRAY(cam_intrinsic);
    p_cams[i]->SetExtrinsicAndIntrinsic(Lidar_T_wrt_Cam, Lidar_R_wrt_Cam,
                                        Lidar_T_wrt_IMU, Lidar_R_wrt_IMU,
                                        cam_intrinsic_mat);
  }
}

void LaserMappingNode::timer_callback() {
  init_cam_process();
  if (sync_packages(Measures, p_cams)) {
    if (flg_first_scan) {
      first_lidar_time = Measures.lidar_beg_time;
      p_imu->first_lidar_time = first_lidar_time;
      flg_first_scan = false;
      return;
    }

    double t0, t1, t2, t3, t4, t5, t6, t7, match_start, solve_start, svd_time;

    match_time = 0;
    kdtree_search_time = 0.0;
    solve_time = 0;
    solve_const_H_time = 0;
    svd_time = 0;

    t0 = omp_get_wtime();

    p_imu->Process(Measures, kf, feats_undistort, p_cams);
    state_point = kf.get_x();
    pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

    t1 = omp_get_wtime();
    imu_time = t1 - t0;

    if (feats_undistort->empty() || (feats_undistort == NULL)) {
      RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
      return;
    }

    flg_EKF_inited =
        (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;
    /*** Segment the map in lidar FOV ***/
    // lasermap_fov_segment();

    std::vector<float> N_dst;
    std::vector<int> added_idxs, new_idxs, N_idxs, filter_idxs;

    scan_min_extent = 0.01 * p_pre->mean_range;
    map_bucket_size =
        1 + floor((1.0 - fmin(1.0, scan_min_extent / filter_size_map_min)) *
                  MAX_NEIGHBOURS);
    map_search_radius = fmin(filter_size_corner_min, 10 * scan_min_extent);

    std::cerr << "Min extent: " << scan_min_extent << std::endl;
    std::cerr << "Bucket size: " << map_bucket_size << std::endl;
    std::cerr << "Search radius: " << map_search_radius << std::endl;

    /*** initialize the map kdtree ***/
    // if (ikdtree.Root_Node == nullptr) {
    if (ioctree.size() == 0) {
      RCLCPP_INFO(this->get_logger(), "Initialize the map kdtree");
      if (feats_undistort->points.size() < NUM_MATCH_POINTS) return;
      map_incremental(true);
      return;
    }

    /*** downsample the feature points in a scan ***/
    ioctree_scan.set_min_extent(scan_min_extent);
    ioctree_scan.initialize(*feats_undistort, added_idxs, new_idxs);

    std::cerr << "Scan size: " << feats_undistort->size() << std::endl;
    std::cerr << "Downsample size: " << added_idxs.size() << std::endl;

    *feats_down_body = FastLioPointCloud(*feats_undistort, added_idxs);
    feats_down_size = feats_down_body->points.size();

    t2 = omp_get_wtime();
    downsample_time = t2 - t1;

    int featsFromMapNum = ioctree.size();  // ikdtree.validnum();
    kdtree_size_st = ioctree.size();       // ikdtree.size();

    t3 = omp_get_wtime();
    init_kdtree_time = t3 - t2;

    /*** ICP and iterated Kalman filter update ***/
    if (feats_down_size < 5) {
      RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
      return;
    }

    normvec->resize(feats_down_size);
    feats_down_world->resize(feats_down_size);

    V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
    fout_pre << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
             << euler_cur.transpose() << " " << state_point.pos.transpose()
             << " " << ext_euler.transpose() << " "
             << state_point.offset_T_L_I.transpose() << " "
             << state_point.vel.transpose() << " " << state_point.bg.transpose()
             << " " << state_point.ba.transpose() << " " << state_point.grav
             << endl;

    pointSearchInd_surf.resize(feats_down_size);
    Nearest_Points.resize(feats_down_size);
    int rematch_num = 0;
    bool nearest_search_en = true;

    /*** iterated state estimation ***/
    t4 = omp_get_wtime();
    double t_update_start = omp_get_wtime();
    double solve_H_time = 0;
    kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
    state_point = kf.get_x();
    euler_cur = SO3ToEuler(state_point.rot);
    pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
    geoQuat.x = state_point.rot.coeffs()[0];
    geoQuat.y = state_point.rot.coeffs()[1];
    geoQuat.z = state_point.rot.coeffs()[2];
    geoQuat.w = state_point.rot.coeffs()[3];
    publish_odometry();

    double t_update_end = omp_get_wtime();
    t5 = omp_get_wtime();
    state_update_time = t5 - t4;

    /*** add the feature points to map kdtree ***/
    map_incremental(false);
    t6 = omp_get_wtime();
    kdtree_update_time = t6 - t5;
    t7 = omp_get_wtime();
    total_time = t7 - t0;

    double eigen_time = t7 - t6;
    // std::cerr << "Eigen decomposition time: " << eigen_time <<
    // std::endl;

    /*** Debug variables ***/
    if (runtime_pos_log) {
      frame_num++;
      kdtree_size_end = ioctree.size();  // ikdtree.size();
      aver_time_consu =
          aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
      aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num +
                      (t_update_end - t_update_start) / frame_num;
      aver_time_match = aver_time_match * (frame_num - 1) / frame_num +
                        (match_time) / frame_num;
      aver_time_incre = aver_time_incre * (frame_num - 1) / frame_num +
                        (kdtree_incremental_time) / frame_num;
      aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num +
                        (solve_time + solve_H_time) / frame_num;
      aver_time_const_H_time =
          aver_time_const_H_time * (frame_num - 1) / frame_num +
          solve_time / frame_num;
      max_time_consu = fmax(max_time_consu, t5 - t0);
      max_time_icp = fmax(max_time_icp, t_update_end - t_update_start);
      max_time_match = fmax(max_time_match, match_time);
      max_time_incre = fmax(max_time_incre, kdtree_incremental_time);
      max_time_solve = fmax(max_time_solve, solve_time + solve_H_time);
      max_time_const_H_time = fmax(max_time_const_H_time, solve_time);
      max_imu_time = fmax(max_imu_time, imu_time);
      max_init_kdtree_time = fmax(max_init_kdtree_time, init_kdtree_time);
      max_state_update_time = fmax(max_state_update_time, state_update_time);
      max_kdtree_update_time = fmax(max_kdtree_update_time, kdtree_update_time);
      max_downsample_time = fmax(max_downsample_time, downsample_time);
      max_total_time = fmax(max_total_time, total_time);
      T1[time_log_counter] = Measures.lidar_beg_time;
      s_plot[time_log_counter] = t5 - t0;
      s_plot2[time_log_counter] = feats_undistort->points.size();
      s_plot3[time_log_counter] = kdtree_incremental_time;
      s_plot4[time_log_counter] = kdtree_search_time;
      s_plot5[time_log_counter] = kdtree_delete_counter;
      s_plot6[time_log_counter] = kdtree_delete_time;
      s_plot7[time_log_counter] = kdtree_size_st;
      s_plot8[time_log_counter] = kdtree_size_end;
      s_plot9[time_log_counter] = aver_time_consu;
      s_plot10[time_log_counter] = add_point_size;
      time_log_counter++;

      printf(
          "IMU: %0.6f Downsample: %0.6f Init kdtree: %0.6f "
          "State update: %0.6f Kdtree update: %0.6f "
          "Total: %0.6f\n",
          imu_time, downsample_time, init_kdtree_time, state_update_time,
          kdtree_update_time, total_time);
      printf(
          "Max IMU: %0.6f Max downsample: %0.6f Max init kdtree: %0.6f "
          "Max state update: %0.6f Max kdtree update: "
          "%0.6f Max total time: %0.6f\n",
          max_imu_time, max_downsample_time, max_init_kdtree_time,
          max_state_update_time, max_kdtree_update_time, max_total_time);

      ext_euler = SO3ToEuler(state_point.offset_R_L_I);
      fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
               << euler_cur.transpose() << " " << state_point.pos.transpose()
               << " " << ext_euler.transpose() << " "
               << state_point.offset_T_L_I.transpose() << " "
               << state_point.vel.transpose() << " "
               << state_point.bg.transpose() << " "
               << state_point.ba.transpose() << " " << state_point.grav << " "
               << feats_undistort->points.size() << endl;
      dump_lio_state_to_log(fp);
    }
  }
}

void LaserMappingNode::map_save_callback(
    std_srvs::srv::Trigger::Request::ConstSharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res) {
  RCLCPP_INFO(this->get_logger(), "Saving map to %s...", map_file_path.c_str());
  if (pcd_save_en) {
    save_to_pcd();
    res->success = true;
    res->message = "Map saved.";
  } else {
    res->success = false;
    res->message = "Map save disabled.";
  }
}
}  // namespace fastlio

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(fastlio::LaserMappingNode)