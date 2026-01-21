#pragma once

#ifndef USE_IKFOM_H
#define USE_IKFOM_H

#include <IKFoM_toolkit/esekfom/esekfom.hpp>

typedef MTK::vect<3, double> vect3;
typedef MTK::SO3<double> SO3;
typedef MTK::S2<double, 98090, 10000, 1> S2;
typedef MTK::vect<1, double> vect1;
typedef MTK::vect<2, double> vect2;

MTK_BUILD_MANIFOLD(state_ikfom,
                   ((vect3, pos))((SO3, rot))((SO3, offset_R_L_I))(
                       (vect3, offset_T_L_I))((vect3, linvel))((vect3, angvel))(
                       (vect3, bg))((vect3, ba))((S2, grav)));

MTK_BUILD_MANIFOLD(input_ikfom, ((vect3, acc))((vect3, gyro)));

MTK_BUILD_MANIFOLD(process_noise_ikfom,
                   ((vect3, ng))((vect3, na))((vect3, nbg))((vect3, nba)));

typedef esekfom::esekf<state_ikfom, 12, input_ikfom> Ikfom;
typedef std::shared_ptr<Ikfom> IkfomSPtr;

inline Ikfom::cov P_cov() {
  Ikfom::cov cov;
  cov.setIdentity();
  cov *= 1e-3;
  return cov;
}

inline MTK::get_cov<process_noise_ikfom>::type process_noise_cov() {
  MTK::get_cov<process_noise_ikfom>::type cov =
      MTK::get_cov<process_noise_ikfom>::type::Zero();
  MTK::setDiagonal<process_noise_ikfom, vect3, 0>(cov, &process_noise_ikfom::ng,
                                                  1e-3);
  MTK::setDiagonal<process_noise_ikfom, vect3, 3>(cov, &process_noise_ikfom::na,
                                                  1e-3);
  MTK::setDiagonal<process_noise_ikfom, vect3, 6>(
      cov, &process_noise_ikfom::nbg, 1e-3);
  MTK::setDiagonal<process_noise_ikfom, vect3, 9>(
      cov, &process_noise_ikfom::nba, 1e-3);
  return cov;
}

inline Eigen::Matrix<double, 27, 1> get_f(state_ikfom& s,
                                          const input_ikfom& in) {
  Eigen::Matrix<double, 27, 1> res = Eigen::Matrix<double, 27, 1>::Zero();
  vect3 omega;
  in.gyro.boxminus(omega, s.bg);
  vect3 a_inertial = s.rot * (in.acc - s.ba);
  for (int i = 0; i < 3; i++) {
    res(i) = s.linvel[i];
    res(i + 3) = omega[i];
    res(i + 12) = a_inertial[i] + s.grav[i];
    res(i + 15) = omega[i];
  }
  return res;
}

inline Eigen::Matrix<double, 27, 26> df_dx(state_ikfom& s,
                                           const input_ikfom& in) {
  Eigen::Matrix<double, 27, 26> cov = Eigen::Matrix<double, 27, 26>::Zero();
  cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
  vect3 acc_;
  in.acc.boxminus(acc_, s.ba);
  vect3 omega;
  in.gyro.boxminus(omega, s.bg);
  cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix() * MTK::hat(acc_);
  cov.template block<3, 3>(12, 21) = -s.rot.toRotationMatrix();
  Eigen::Matrix<state_ikfom::scalar, 2, 1> vec =
      Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
  Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
  s.S2_Mx(grav_matrix, vec, 24);
  cov.template block<3, 2>(12, 24) = grav_matrix;
  cov.template block<3, 3>(3, 18) = -Eigen::Matrix3d::Identity();
  cov.template block<3, 3>(3, 15) = Eigen::Matrix3d::Identity();
  cov.template block<3, 3>(15, 18) = -Eigen::Matrix3d::Identity();
  return cov;
}

inline Eigen::Matrix<double, 27, 12> df_dw(state_ikfom& s,
                                           const input_ikfom& in) {
  Eigen::Matrix<double, 27, 12> cov = Eigen::Matrix<double, 27, 12>::Zero();
  cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix();
  cov.template block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
  cov.template block<3, 3>(18, 6) = Eigen::Matrix3d::Identity();
  cov.template block<3, 3>(21, 9) = Eigen::Matrix3d::Identity();
  cov.template block<3, 3>(15, 0) = -Eigen::Matrix3d::Identity();
  return cov;
}

inline vect3 SO3ToEuler(const SO3& orient) {
  Eigen::Matrix<double, 3, 1> _ang;
  Eigen::Vector4d q_data = orient.coeffs().transpose();

  double sqw = q_data[3] * q_data[3];
  double sqx = q_data[0] * q_data[0];
  double sqy = q_data[1] * q_data[1];
  double sqz = q_data[2] * q_data[2];
  double unit = sqx + sqy + sqz + sqw;
  double test = q_data[3] * q_data[1] - q_data[2] * q_data[0];

  if (test > 0.49999 * unit) {
    _ang << 2 * std::atan2(q_data[0], q_data[3]), M_PI / 2, 0;
    double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
    vect3 euler_ang(temp, 3);
    return euler_ang;
  }
  if (test < -0.49999 * unit) {
    _ang << -2 * std::atan2(q_data[0], q_data[3]), -M_PI / 2, 0;
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

#endif