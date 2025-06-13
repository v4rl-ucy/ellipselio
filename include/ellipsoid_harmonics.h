#ifndef ELLIPSOIDS_HARMONICS_H
#define ELLIPSOIDS_HARMONICS_H

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <algorithm>
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>
#include <autodiff/forward/utils/gradient.hpp>
#include <cmath>
#include <iostream>
#include <vector>

using Vec3f = Eigen::Vector3f;
using Mat3f = Eigen::Matrix3f;

using autodiff::dual2nd;
using autodiff::detail::hessian;

struct SHCoeffs {
  Eigen::VectorXf weights;
  Eigen::MatrixXf r_coeffs;
  Eigen::MatrixXf g_coeffs;
  Eigen::MatrixXf b_coeffs;

  SHCoeffs() = default;
  SHCoeffs(int p_num, int l_max)
      : weights(p_num),
        r_coeffs(p_num, (l_max + 1) * (l_max + 1)),
        g_coeffs(p_num, (l_max + 1) * (l_max + 1)),
        b_coeffs(p_num, (l_max + 1) * (l_max + 1)) {}
};

class EllipsoidHarmonics {
 public:
  EllipsoidHarmonics(int l_max = 3);

  int getCoefficientCount() const;

  // compute SH coefficients in parallel
  void computeCoefficients(const Vec3f& dir, const Vec3f& color,
                           SHCoeffs& coeffs, int p_idx) const;

  // Finalize coefficients by normalizing with accumulated weight
  void finalizeCoefficients(SHCoeffs& coeffs, Eigen::MatrixXf& sh_mat) const;

  // Evaluate color from direction vector on unit sphere
  Vec3f evaluateColorFromDirection(const Eigen::MatrixXf& sh_mat,
                                   const Vec3f& dir) const;

  // Find direction that best matches a target color using autodiff Newton
  // optimization
  Vec3f findDirectionMatchingColor(const Eigen::MatrixXf& sh_mat,
                                   const Vec3f& target_color,
                                   const Vec3f& initial_dir, int max_iters = 20,
                                   float epsilon = 1e-6f) const;

 private:
  int l_max_;
  int n_coeffs_;

  // Compute the normalization constant for SH
  float K(int l, int m) const;

  // Associated Legendre polynomial
  float P(int l, int m, float x) const;

  // Evaluate SH basis function at (theta, phi)
  float SH(int l, int m, float theta, float phi) const;

  // Convert Cartesian direction to spherical coords (theta, phi)
  Eigen::Vector2f cartesianToSpherical(const Vec3f& dir) const;

  // Map unit direction vector to ellipsoid surface point
  Vec3f ellipsoidPointFromDir(const Vec3f& dir, const Vec3f& scale,
                              const Mat3f& rot) const;

  Vec3f dirFromEllipsoidPoint(const Vec3f& point, const Vec3f& scale,
                              const Mat3f& rot) const;
};

#endif  // ELLIPSOIDS_HARMONICS_H
