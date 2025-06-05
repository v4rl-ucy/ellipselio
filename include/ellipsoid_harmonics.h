#ifndef ELLIPSOIDS_HARMONICS_H
#define ELLIPSOIDS_HARMONICS_H

#include <Eigen/Core>
#include <autodiff/forward/dual2nd.hpp>
#include <vector>

using Vec3f = Eigen::Vector3f;
using SHCoeffs = std::vector<Eigen::VectorXf>;

struct SHAccumulation {
  float weight = 0.0f;
  std::vector<Eigen::VectorXf> raw_coeffs;  // size 3, one per color channel
  std::vector<Eigen::VectorXf> sh_coeffs;
};

class EllipsoidHarmonics {
 public:
  EllipsoidHarmonics(int l_max, bool use_gpu = false);

  // Set ellipsoid axes lengths (a, b, c)
  void setEllipsoidAxes(float a, float b, float c);

  int getCoefficientCount() const;

  // Accumulate SH coefficients incrementally
  void accumulateCoefficients(const std::vector<Vec3f>& directions,
                              const std::vector<Vec3f>& colors,
                              SHAccumulation& accum) const;

  // Finalize coefficients by normalizing with accumulated weight
  void finalizeCoefficients(const SHAccumulation& accum,
                            SHCoeffs& out_coeffs) const;

  // Compute coefficients from scratch (clears previous accum)
  void computeCoefficients(const std::vector<Vec3f>& directions,
                           const std::vector<Vec3f>& colors,
                           SHCoeffs& out_coeffs);

  // Evaluate color from direction vector on unit sphere
  Eigen::Vector3f evaluateColorFromDirection(const SHCoeffs& sh_coeffs,
                                             const Eigen::Vector3f& dir) const;

  // Evaluate color at point on ellipsoid (direction mapped to ellipsoid
  // surface)
  Eigen::Vector3f evaluateColorOnEllipsoidFromDir(const Eigen::Vector3f& dir,
                                                  const SHCoeffs& coeffs) const;

  // Find direction that best matches a target color using autodiff Newton
  // optimization
  Eigen::Vector3f findDirectionMatchingColor(
      const SHCoeffs& coeffs, const Eigen::Vector3f& target_color,
      const Eigen::Vector3f& initial_dir, int max_iters = 20,
      float epsilon = 1e-6f) const;

 private:
  int l_max_;
  int n_coeffs_;
  bool use_gpu_;

  float a_, b_, c_;  // Ellipsoid axes

  // Compute the normalization constant for SH
  float K(int l, int m) const;

  // Associated Legendre polynomial
  float P(int l, int m, float x) const;

  // Evaluate SH basis function at (theta, phi)
  float SH(int l, int m, float theta, float phi) const;

  void computeOnGPU(const std::vector<Vec3f>& directions,
                    const std::vector<Vec3f>& colors, SHCoeffs& out_coeffs);

  // Convert Cartesian direction to spherical coords (theta, phi)
  Eigen::Vector2f cartesianToSpherical(const Eigen::Vector3f& dir) const;

  // Map unit direction vector to ellipsoid surface point
  Eigen::Vector3f ellipsoidPointFromDir(const Eigen::Vector3f& dir) const;
};

#endif  // ELLIPSOIDS_HARMONICS_H
