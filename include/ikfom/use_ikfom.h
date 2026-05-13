#ifndef USE_IKFOM_H_
#define USE_IKFOM_H_

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include "esekfom/esekfom.hpp"
#include "so3_math.h"

inline constexpr double kPi = 3.14159265358979323846;

using vect3 = MTK::vect<3, double>;
using SO3 = MTK::SO3<double>;
using S2 = MTK::S2<double, 98090, 10000, 1>;
using vect1 = MTK::vect<1, double>;
using vect2 = MTK::vect<2, double>;

MTK_BUILD_MANIFOLD(state_ikfom, ((vect3, pos))((SO3, rot))((SO3, offset_R_L_I))(
                                    (vect3, offset_T_L_I))((vect3, vel))(
                                    (vect3, bg))((vect3, ba))((S2, grav)));

MTK_BUILD_MANIFOLD(input_ikfom, ((vect3, acc))((vect3, gyro)));

MTK_BUILD_MANIFOLD(process_noise_ikfom,
                   ((vect3, ng))((vect3, na))((vect3, nbg))((vect3, nba)));

using Ikfom = esekfom::esekf<state_ikfom, 12, input_ikfom>;
using IkfomSPtr = std::shared_ptr<Ikfom>;

inline Ikfom::cov PCov() {
  Ikfom::cov cov;
  cov.setIdentity();
  cov *= 1e-3;
  return cov;
}

inline MTK::get_cov<process_noise_ikfom>::type ProcessNoiseCov() {
  MTK::get_cov<process_noise_ikfom>::type cov =
      MTK::get_cov<process_noise_ikfom>::type::Identity();
  cov *= 1e-3;
  return cov;
}

inline Eigen::Matrix<double, 24, 1> GetF(state_ikfom& s,
                                         const input_ikfom& in) {
  Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
  vect3 omega;
  in.gyro.boxminus(omega, s.bg);
  vect3 a_inertial = s.rot * (in.acc - s.ba);
  for (int i = 0; i < 3; i++) {
    res(i) = s.vel[i];
    res(i + 3) = omega[i];
    res(i + 12) = a_inertial[i] + s.grav[i];
  }
  return res;
}

inline Eigen::Matrix<double, 24, 23> DfDx(state_ikfom& s,
                                          const input_ikfom& in) {
  Eigen::Matrix<double, 24, 23> cov = Eigen::Matrix<double, 24, 23>::Zero();
  cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
  vect3 acc_;
  in.acc.boxminus(acc_, s.ba);
  vect3 omega;
  in.gyro.boxminus(omega, s.bg);
  cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix() * MTK::hat(acc_);
  cov.template block<3, 3>(12, 18) = -s.rot.toRotationMatrix();
  Eigen::Matrix<state_ikfom::scalar, 2, 1> vec =
      Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
  Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
  s.S2_Mx(grav_matrix, vec, 21);
  cov.template block<3, 2>(12, 21) = grav_matrix;
  cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();
  return cov;
}

inline Eigen::Matrix<double, 24, 12> DfDw(state_ikfom& s,
                                          const input_ikfom& in) {
  Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
  cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix();
  cov.template block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
  cov.template block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();
  cov.template block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();
  return cov;
}

inline vect3 So3ToEuler(const SO3& orient) {
  Eigen::Matrix<double, 3, 1> _ang;
  Eigen::Vector4d q_data = orient.coeffs().transpose();

  double sqw = q_data[3] * q_data[3];
  double sqx = q_data[0] * q_data[0];
  double sqy = q_data[1] * q_data[1];
  double sqz = q_data[2] * q_data[2];
  double unit = sqx + sqy + sqz + sqw;
  double test = q_data[3] * q_data[1] - q_data[2] * q_data[0];

  if (test > 0.49999 * unit) {
    _ang << 2 * std::atan2(q_data[0], q_data[3]), kPi / 2, 0;
    double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
    vect3 euler_ang(temp, 3);
    return euler_ang;
  }
  if (test < -0.49999 * unit) {
    _ang << -2 * std::atan2(q_data[0], q_data[3]), -kPi / 2, 0;
    double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
    vect3 euler_ang(temp, 3);
    return euler_ang;
  }

  _ang << std::atan2(2 * q_data[0] * q_data[3] + 2 * q_data[1] * q_data[2],
                     -sqx - sqy + sqz + sqw),
      std::asin(2 * test / unit),
      std::atan2(2 * q_data[2] * q_data[3] + 2 * q_data[1] * q_data[0],
                 sqx - sqy - sqz + sqw);
  double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
  vect3 euler_ang(temp, 3);

  return euler_ang;
}

#endif  // USE_IKFOM_H_