#include <algorithm>
#include <cmath>
#include <iostream>

#include "SphericalHarmonicsProjector.h"

constexpr float PI = 3.14159265359f;

SphericalHarmonicsProjector::SphericalHarmonicsProjector(int l_max, float a,
                                                         float b, float c)
    : l_max_(l_max),
      n_coeffs_((l_max + 1) * (l_max + 1)),
      a_(a),
      b_(b),
      c_(c) {}

int SphericalHarmonicsProjector::getCoefficientCount() const {
  return n_coeffs_;
}

float SphericalHarmonicsProjector::K(int l, int m) const {
  return std::sqrt((2 * l + 1) * std::tgamma(l - m + 1) /
                   (4 * PI * std::tgamma(l + m + 1)));
}

float SphericalHarmonicsProjector::P(int l, int m, float x) const {
  float pmm = 1.0f;
  if (m > 0) {
    float somx2 = std::sqrt((1.0f - x) * (1.0f + x));
    float fact = 1.0f;
    for (int i = 1; i <= m; ++i) {
      pmm *= -fact * somx2;
      fact += 2.0f;
    }
  }

  if (l == m) return pmm;

  float pmmp1 = x * (2 * m + 1) * pmm;
  if (l == m + 1) return pmmp1;

  float pll = 0.0f;
  for (int ll = m + 2; ll <= l; ++ll) {
    pll = ((2 * ll - 1) * x * pmmp1 - (ll + m - 1) * pmm) / (ll - m);
    pmm = pmmp1;
    pmmp1 = pll;
  }
  return pll;
}

float SphericalHarmonicsProjector::SH(int l, int m, float theta,
                                      float phi) const {
  if (m > 0)
    return std::sqrt(2.0f) * K(l, m) * std::cos(m * phi) *
           P(l, m, std::cos(theta));
  if (m < 0)
    return std::sqrt(2.0f) * K(l, -m) * std::sin(-m * phi) *
           P(l, -m, std::cos(theta));
  return K(l, 0) * P(l, 0, std::cos(theta));
}

Vec3f SphericalHarmonicsProjector::mapToUnitSphere(const Vec3f& dir) const {
  float x = dir.x() / a_;
  float y = dir.y() / b_;
  float z = dir.z() / c_;
  Vec3f scaled(x, y, z);
  return scaled.normalized();
}

void SphericalHarmonicsProjector::computeCoefficients(
    const std::vector<Vec3f>& directions, const std::vector<Vec3f>& colors,
    SHCoeffs& out_coeffs) {
  out_coeffs.resize(3);
  for (auto& c : out_coeffs) {
    c = Eigen::VectorXf::Zero(n_coeffs_);
  }

  float total_weight = 0.0f;

  for (size_t i = 0; i < directions.size(); ++i) {
    Vec3f dir = mapToUnitSphere(directions[i]);
    const auto& color = colors[i];

    float theta = std::acos(std::clamp(dir.z(), -1.0f, 1.0f));
    float phi = std::atan2(dir.y(), dir.x());
    if (phi < 0.0f) phi += 2 * PI;

    float weight = std::sin(theta);
    total_weight += weight;

    int idx = 0;
    for (int l = 0; l <= l_max_; ++l) {
      for (int m = -l; m <= l; ++m) {
        float ylm = SH(l, m, theta, phi);
        out_coeffs[0](idx) += color.x() * ylm * weight;
        out_coeffs[1](idx) += color.y() * ylm * weight;
        out_coeffs[2](idx) += color.z() * ylm * weight;
        ++idx;
      }
    }
  }

  for (auto& c : out_coeffs) {
    c /= total_weight;
  }
}

Eigen::Vector3f SphericalHarmonicsProjector::findClosestDirectionToColorNewton(
    const SHCoeffs& sh_coeffs, const Eigen::Vector3f& target_color,
    const Eigen::Vector3f& initial_dir, int max_iters, float epsilon) const {
  Vec3f sph_dir = mapToUnitSphere(initial_dir);
  auto sph = cartesianToSpherical(sph_dir);
  dual2nd theta = sph[0];
  dual2nd phi = sph[1];

  for (int iter = 0; iter < max_iters; ++iter) {
    auto loss_fn = [&](const auto& vars) {
      dual2nd t = vars(0), p = vars(1);
      VectorX<dual2nd> Y(n_coeffs_);

      int idx = 0;
      for (int l = 0; l <= l_max_; ++l)
        for (int m = -l; m <= l; ++m) Y(idx++) = SH(l, m, t, p);

      dual2nd loss = 0.0;
      for (int c = 0; c < 3; ++c) {
        auto col = sh_coeffs[c].cast<dual2nd>().dot(Y);
        loss += pow(col - target_color[c], 2);
      }
      return loss;
    };

    Eigen::Vector2<dual2nd> vars(theta, phi);
    dual2nd loss;
    Eigen::Vector2d grad;
    Eigen::Matrix2d hess;

    loss = hessian(loss_fn, wrt(vars), at(vars), grad, hess);

    Eigen::Vector2d delta = -hess.ldlt().solve(grad);

    theta = theta + delta(0);
    phi = phi + delta(1);

    theta = std::clamp(theta, 0.001, PI - 0.001);
    phi = std::fmod(phi, 2 * PI);
    if (phi < 0.0) phi += 2 * PI;

    if (grad.norm() < epsilon || delta.norm() < epsilon) break;
  }

  float th = val(theta);
  float ph = val(phi);
  Eigen::Vector3f u(std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph),
                    std::cos(th));
  Eigen::Vector3f e(a_ * u.x(), b_ * u.y(), c_ * u.z());
  return e.normalized();
}

Eigen::Vector3f SphericalHarmonicsProjector::evaluateColorFromDirection(
    const SHCoeffs& sh_coeffs, const Eigen::Vector3f& dir) const {
  Vec3f sph_dir = mapToUnitSphere(dir);
  float x = sph_dir.x(), y = sph_dir.y(), z = sph_dir.z();
  float r = std::max(sph_dir.norm(), 1e-8f);
  float theta = std::acos(std::clamp(z / r, -1.0f, 1.0f));
  float phi = std::atan2(y, x);
  if (phi < 0.0f) phi += 2.0f * PI;

  Eigen::VectorXf Y(n_coeffs_);
  int idx = 0;
  for (int l = 0; l <= l_max_; ++l)
    for (int m = -l; m <= l; ++m) Y(idx++) = val(SH(l, m, theta, phi));

  Eigen::Vector3f color;
  for (int c = 0; c < 3; ++c) color[c] = sh_coeffs[c].dot(Y);

  return color;
}
