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
using autodiff::dual2nd;
using autodiff::detail::hessian;

struct SHCoeffs {
  float weight = 0.0f;
  Eigen::MatrixXf raw_coeffs;  // size 3, one per color channel
  Eigen::MatrixXf sh_coeffs;
};

class EllipsoidHarmonics {
 public:
  EllipsoidHarmonics(int l_max = 3);

  int getCoefficientCount() const;

  // Accumulate SH coefficients incrementally
  void accumulateCoefficients(const std::vector<Vec3f>& directions,
                              const std::vector<Vec3f>& colors,
                              SHCoeffs& coeffs) const;

  // Finalize coefficients by normalizing with accumulated weight
  void finalizeCoefficients(SHCoeffs& coeffs) const;

  // Evaluate color from direction vector on unit sphere
  Eigen::Vector3f evaluateColorFromDirection(const SHCoeffs& coeffs,
                                             const Eigen::Vector3f& dir) const;

  // Find direction that best matches a target color using autodiff Newton
  // optimization
  Eigen::Vector3f findDirectionMatchingColor(
      const SHCoeffs& coeffs, const Eigen::Vector3f& target_color,
      const Eigen::Vector3f& initial_dir, int max_iters = 20,
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
  Eigen::Vector2f cartesianToSpherical(const Eigen::Vector3f& dir) const;

  // Map unit direction vector to ellipsoid surface point
  Eigen::Vector3f ellipsoidPointFromDir(const Eigen::Vector3f& dir,
                                        const Eigen::Vector3f& scale) const;

  Eigen::Vector3f dirFromEllipsoidPoint(const Eigen::Vector3f& point,
                                        const Eigen::Vector3f& scale) const;
};

#endif  // ELLIPSOIDS_HARMONICS_H
