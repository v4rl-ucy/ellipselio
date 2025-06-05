#pragma once
#include <Eigen/Dense>
#include <autodiff/forward/dual2nd.hpp>
#include <autodiff/forward/dual2nd/eigen.hpp>
#include <vector>
using namespace autodiff;

class EllipsoidHarmonicsProjector {
 public:
  using Vec3f = Eigen::Vector3f;
  using SHCoeffs = std::vector<Eigen::VectorXf>;  // R, G, B

  explicit EllipsoidHarmonicsProjector(int l_max);

  void computeCoefficients(const std::vector<Vec3f>& directions,
                           const std::vector<Vec3f>& colors,
                           SHCoeffs& out_coeffs);

  int getCoefficientCount() const;

 private:
  int l_max_;
  int n_coeffs_;

  float SH(int l, int m, float theta, float phi) const;
  float P(int l, int m, float x) const;
  float K(int l, int m) const;

  Eigen::Vector3f findClosestDirectionToColorNewton(
      const SHCoeffs& sh_coeffs, const Eigen::Vector3f& target_color,
      const Eigen::Vector3f& initial_dir,  // new!
      int max_iters = 20, float epsilon = 1e-6f) const;
};

auto cartesianToSphere = [](const Eigen::Vector3f& dir) -> Eigen::Vector2d {
  float x = dir.x(), y = dir.y(), z = dir.z();
  float r = std::max(dir.norm(), 1e-8f);                    // avoid divide by 0
  float theta = std::acos(std::clamp(z / r, -1.0f, 1.0f));  // [0, π]
  float phi = std::atan2(y, x);                             // [-π, π]
  if (phi < 0.0f) phi += 2.0f * M_PI;                       // [0, 2π]
  return {theta, phi};
};

Eigen::Vector3f evaluateColorFromDirection(const SHCoeffs& sh_coeffs,
                                           const Eigen::Vector3f& dir) const;