#pragma once

#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <common_pcl.h>
#include <so3_math.h>

#include <Eigen/Eigen>
#include <deque>
#include <fast_lio/msg/pose6_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

using namespace std;
using namespace Eigen;

#define USE_IKFOM

#define PI_M (3.14159265358)
#define G_m_s2 (9.81)    // Gravaty const in GuangDong/China
#define DIM_STATE (18)   // Dimension of states (Let Dim(SO(3)) = 3)
#define DIM_PROC_N (12)  // Dimension of process noise (Let Dim(SO(3)) = 3)
#define CUBE_LEN (6.0)
#define LIDAR_SP_LEN (2)
#define INIT_COV (1)
#define NUM_MATCH_POINTS (5)
#define MAX_MEAS_DIM (10000)

#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
#define CONSTRAIN(v, min, max) ((v > min) ? ((v < max) ? v : max) : min)
#define ARRAY_FROM_EIGEN(mat) mat.data(), mat.data() + mat.rows() * mat.cols()
#define STD_VEC_FROM_EIGEN(mat)             \
  vector<decltype(mat)::Scalar>(mat.data(), \
                                mat.data() + mat.rows() * mat.cols())
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "Log/" + name))

typedef fast_lio::msg::Pose6D Pose6D;
typedef Vector3d V3D;
typedef Matrix3d M3D;
typedef Vector3f V3F;
typedef Matrix3f M3F;

#define MD(a, b) Matrix<double, (a), (b)>
#define VD(a) Matrix<double, (a), 1>
#define MF(a, b) Matrix<float, (a), (b)>
#define VF(a) Matrix<float, (a), 1>

static M3D Eye3d(M3D::Identity());
static M3F Eye3f(M3F::Identity());
static V3D Zero3d(0, 0, 0);
static V3F Zero3f(0, 0, 0);

struct MeasureGroup  // Lidar data and imu dates for the curent process
{
  MeasureGroup() {
    lidar_beg_time = 0.0;
    this->lidar.reset(new FastLioPointCloud());
  };
  double lidar_beg_time;
  double lidar_end_time;
  FastLioPointCloud::Ptr lidar;
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu;
};

struct StatesGroup {
  StatesGroup() {
    this->rot_end = M3D::Identity();
    this->pos_end = Zero3d;
    this->vel_end = Zero3d;
    this->bias_g = Zero3d;
    this->bias_a = Zero3d;
    this->gravity = Zero3d;
    this->cov = MD(DIM_STATE, DIM_STATE)::Identity() * INIT_COV;
    this->cov.block<9, 9>(9, 9) = MD(9, 9)::Identity() * 0.00001;
  };

  StatesGroup(const StatesGroup &b) {
    this->rot_end = b.rot_end;
    this->pos_end = b.pos_end;
    this->vel_end = b.vel_end;
    this->bias_g = b.bias_g;
    this->bias_a = b.bias_a;
    this->gravity = b.gravity;
    this->cov = b.cov;
  };

  StatesGroup &operator=(const StatesGroup &b) {
    this->rot_end = b.rot_end;
    this->pos_end = b.pos_end;
    this->vel_end = b.vel_end;
    this->bias_g = b.bias_g;
    this->bias_a = b.bias_a;
    this->gravity = b.gravity;
    this->cov = b.cov;
    return *this;
  };

  StatesGroup operator+(const Matrix<double, DIM_STATE, 1> &state_add) {
    StatesGroup a;
    a.rot_end =
        this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
    a.pos_end = this->pos_end + state_add.block<3, 1>(3, 0);
    a.vel_end = this->vel_end + state_add.block<3, 1>(6, 0);
    a.bias_g = this->bias_g + state_add.block<3, 1>(9, 0);
    a.bias_a = this->bias_a + state_add.block<3, 1>(12, 0);
    a.gravity = this->gravity + state_add.block<3, 1>(15, 0);
    a.cov = this->cov;
    return a;
  };

  StatesGroup &operator+=(const Matrix<double, DIM_STATE, 1> &state_add) {
    this->rot_end =
        this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
    this->pos_end += state_add.block<3, 1>(3, 0);
    this->vel_end += state_add.block<3, 1>(6, 0);
    this->bias_g += state_add.block<3, 1>(9, 0);
    this->bias_a += state_add.block<3, 1>(12, 0);
    this->gravity += state_add.block<3, 1>(15, 0);
    return *this;
  };

  Matrix<double, DIM_STATE, 1> operator-(const StatesGroup &b) {
    Matrix<double, DIM_STATE, 1> a;
    M3D rotd(b.rot_end.transpose() * this->rot_end);
    a.block<3, 1>(0, 0) = Log(rotd);
    a.block<3, 1>(3, 0) = this->pos_end - b.pos_end;
    a.block<3, 1>(6, 0) = this->vel_end - b.vel_end;
    a.block<3, 1>(9, 0) = this->bias_g - b.bias_g;
    a.block<3, 1>(12, 0) = this->bias_a - b.bias_a;
    a.block<3, 1>(15, 0) = this->gravity - b.gravity;
    return a;
  };

  void resetpose() {
    this->rot_end = M3D::Identity();
    this->pos_end = Zero3d;
    this->vel_end = Zero3d;
  }

  M3D rot_end;  // the estimated attitude (rotation matrix) at the end lidar
                // point
  V3D pos_end;  // the estimated position at the end lidar point (world frame)
  V3D vel_end;  // the estimated velocity at the end lidar point (world frame)
  V3D bias_g;   // gyroscope bias
  V3D bias_a;   // accelerator bias
  V3D gravity;  // the estimated gravity acceleration
  Matrix<double, DIM_STATE, DIM_STATE> cov;  // states covariance
};

template <typename T>
T rad2deg(T radians) {
  return radians * 180.0 / PI_M;
}

template <typename T>
T deg2rad(T degrees) {
  return degrees * PI_M / 180.0;
}

template <typename T>
auto set_pose6d(const double t, const Matrix<T, 3, 1> &a,
                const Matrix<T, 3, 1> &g, const Matrix<T, 3, 1> &v,
                const Matrix<T, 3, 1> &p, const Matrix<T, 3, 3> &R) {
  Pose6D rot_kp;
  rot_kp.offset_time = t;
  for (int i = 0; i < 3; i++) {
    rot_kp.acc[i] = a(i);
    rot_kp.gyr[i] = g(i);
    rot_kp.vel[i] = v(i);
    rot_kp.pos[i] = p(i);
    for (int j = 0; j < 3; j++) rot_kp.rot[i * 3 + j] = R(i, j);
  }
  return move(rot_kp);
}

/* comment
plane equation: Ax + By + Cz + D = 0
convert to: A/D*x + B/D*y + C/D*z = -1
solve: A0*x0 = b0
where A0_i = [x_i, y_i, z_i], x0 = [A/D, B/D, C/D]^T, b0 = [-1, ..., -1]^T
normvec:  normalized x0
*/
template <typename T>
bool esti_normvector(Matrix<T, 3, 1> &normvec, const PointVector &point,
                     const T &threshold, const int &point_num) {
  MatrixXf A(point_num, 3);
  MatrixXf b(point_num, 1);
  b.setOnes();
  b *= -1.0f;

  for (int j = 0; j < point_num; j++) {
    A(j, 0) = point[j].x;
    A(j, 1) = point[j].y;
    A(j, 2) = point[j].z;
  }
  normvec = A.colPivHouseholderQr().solve(b);

  for (int j = 0; j < point_num; j++) {
    if (fabs(normvec(0) * point[j].x + normvec(1) * point[j].y +
             normvec(2) * point[j].z + 1.0f) > threshold) {
      return false;
    }
  }

  normvec.normalize();
  return true;
}

inline float calc_dist(FastLioPoint p1, FastLioPoint p2) {
  float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
            (p1.z - p2.z) * (p1.z - p2.z);
  return d;
}

template <typename T>
bool esti_plane(Matrix<T, 4, 1> &pca_result, const PointVector &point,
                const T &threshold) {
  Matrix<T, NUM_MATCH_POINTS, 3> A;
  Matrix<T, NUM_MATCH_POINTS, 1> b;
  A.setZero();
  b.setOnes();
  b *= -1.0f;

  for (int j = 0; j < NUM_MATCH_POINTS; j++) {
    A(j, 0) = point[j].x;
    A(j, 1) = point[j].y;
    A(j, 2) = point[j].z;
  }

  Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

  T n = normvec.norm();
  pca_result(0) = normvec(0) / n;
  pca_result(1) = normvec(1) / n;
  pca_result(2) = normvec(2) / n;
  pca_result(3) = 1.0 / n;

  for (int j = 0; j < NUM_MATCH_POINTS; j++) {
    if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y +
             pca_result(2) * point[j].z + pca_result(3)) > threshold) {
      return false;
    }
  }
  return true;
}

inline bool esti_color_grad(VF(4) & col_result, VF(4) & pabcd,
                            const PointVector &neighbours,
                            const FastLioPoint &point) {
  M3F QR_mat, QP_mat;
  M3F::Index r_idx, g_idx, b_idx;
  float n_d, p_d, q_d, qp_t, pr_d;
  VF(NUM_MATCH_POINTS) qp_d, qr_d, qr_dot, pq_norm;
  MF(NUM_MATCH_POINTS, 3) QP, QR, QR_tmp;
  V3F n, p, p_c, p_proj, q, q_c, q_proj, qp_proj, r_c, pr_c, qr_c, pr_proj,
      pr_vec, result, qr_vec, qp_vec, qp_scale;

  QP.setZero();
  QR.setZero();
  QR_tmp.setZero();
  qp_d.setZero();
  qr_d.setZero();
  qr_dot.setZero();
  pq_norm.setZero();

  if (!point.has_color) {
    return false;
  }

  qp_t = 0.0;
  r_c << 0, 0, 0;
  n << pabcd(0), pabcd(1), pabcd(2);
  n_d = pabcd(3);

  p = point.getVector3fMap();
  p_c = point.getRGBVector3i().cast<float>() / 255.0;
  p_d = pabcd(0) * p(0) + pabcd(1) * p(1) + pabcd(2) * p(2) + pabcd(3);
  p_proj = p - p_d * n;

  for (int j = 0; j < NUM_MATCH_POINTS; j++) {
    if (!neighbours[j].has_color) {
      return false;
    }

    q = neighbours[j].getVector3fMap();
    q_c = neighbours[j].getRGBVector3i().cast<float>() / 255.0;
    if ((p_c - q_c).norm() < 1.00) {
      q_d = pabcd(0) * q(0) + pabcd(1) * q(1) + pabcd(2) * q(2) + pabcd(3);
      q_proj = q - q_d * n;
      qp_proj = q_proj - p_proj;
      QP.row(j) = qp_proj.normalized();
      qp_d(j) = qp_proj.norm();
      qp_t += 1.0 / qp_d(j);
      r_c += (1.0 / qp_d(j)) * q_c;
      pq_norm(j) = 1;
    }
  }

  if (pq_norm.sum() < 3) {
    // std::cerr << "Not enough similar color points" << std::endl;
    return false;
  }

  r_c *= 1.0 / qp_t;
  pr_c = p_c - r_c;
  pr_d = pr_c.norm();
  pr_c.normalize();

  for (int j = 0; j < NUM_MATCH_POINTS; j++) {
    if (pq_norm(j) == 0) {
      continue;
    }
    q_c = neighbours[j].getRGBVector3i().cast<float>() / 255.0;
    qr_c = q_c - r_c;
    QR.row(j) = qr_c.normalized();
    qr_d(j) = qr_c.norm();
  }

  qr_dot = QR * pr_c;

  qr_dot.maxCoeff(&r_idx);
  if (qr_dot(r_idx) <= 0) {
    return false;
  }
  qr_dot(r_idx) = -1;
  qr_dot.maxCoeff(&g_idx);
  if (qr_dot(g_idx) <= 0) {
    return false;
  }
  qr_dot(g_idx) = -1;
  qr_dot.maxCoeff(&b_idx);
  if (qr_dot(b_idx) <= 0) {
    return false;
  }

  QR_mat.col(0) = QR.row(r_idx);
  QR_mat.col(1) = QR.row(g_idx);
  QR_mat.col(2) = QR.row(b_idx);
  qr_vec = pr_c;

  result = QR_mat.colPivHouseholderQr().solve(qr_vec);

  if (!(QR_mat * pr_d * result).isApprox(pr_d * qr_vec, 0.1f)) {
    return false;
  }

  QP_mat.col(0) = QP.row(r_idx);
  QP_mat.col(1) = QP.row(g_idx);
  QP_mat.col(2) = QP.row(b_idx);
  qp_scale(0) = qp_d(r_idx) / qr_d(r_idx);
  qp_scale(1) = qp_d(g_idx) / qr_d(g_idx);
  qp_scale(2) = qp_d(b_idx) / qr_d(b_idx);
  qp_vec = qp_scale.cwiseProduct(pr_d * result);

  // if (qp_vec.minCoeff() < 0.0 || qp_vec.maxCoeff() > 0.1) {
  //   return false;
  // }

  pr_proj = QP_mat * qp_vec;
  pr_vec = p - p_proj + pr_proj;
  if (pr_vec.normalized().dot(n) < 0.0) {
    col_result << -pr_vec.normalized(), -pr_vec.norm();
  } else {
    col_result << pr_vec.normalized(), pr_vec.norm();
  }

  return true;
}

inline double get_time_sec(const builtin_interfaces::msg::Time &time) {
  return rclcpp::Time(time).seconds();
}

inline rclcpp::Time get_ros_time(double timestamp) {
  int32_t sec = std::floor(timestamp);
  auto nanosec_d = (timestamp - std::floor(timestamp)) * 1e9;
  uint32_t nanosec = nanosec_d;
  return rclcpp::Time(sec, nanosec);
}

#endif