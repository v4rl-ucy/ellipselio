#include <ellipsoid_harmonics.h>

#include <algorithm>
#include <cmath>
#include <iostream>

constexpr float PI = 3.14159265359f;

EllipsoidHarmonics::EllipsoidHarmonics(int l_max)
    : l_max_(l_max), n_coeffs_((l_max + 1) * (l_max + 1)) {}

int EllipsoidHarmonics::getCoefficientCount() const { return n_coeffs_; }

float EllipsoidHarmonics::K(int l, int m) const {
  return std::sqrt((2 * l + 1) * std::tgamma(l - m + 1) /
                   (4 * PI * std::tgamma(l + m + 1)));
}

float EllipsoidHarmonics::P(int l, int m, float x) const {
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

float EllipsoidHarmonics::SH(int l, int m, float theta, float phi) const {
  if (m > 0)
    return std::sqrt(2.0f) * K(l, m) * std::cos(m * phi) *
           P(l, m, std::cos(theta));
  if (m < 0)
    return std::sqrt(2.0f) * K(l, -m) * std::sin(-m * phi) *
           P(l, -m, std::cos(theta));
  return K(l, 0) * P(l, 0, std::cos(theta));
}

void EllipsoidHarmonics::accumulateCoefficients(
    const std::vector<Vec3f>& directions, const std::vector<Vec3f>& colors,
    SHCoeffs& coeffs) const {
  if (!coeffs.raw_coeffs.rows()) {
    coeffs.raw_coeffs.resize(3, n_coeffs_);
    coeffs.sh_coeffs.resize(3, n_coeffs_);
    coeffs.raw_coeffs.setZero();
    coeffs.weight = 0.0f;
  }
  for (size_t i = 0; i < directions.size(); ++i) {
    const auto& dir = directions[i].normalized();
    const auto& color = colors[i];

    float theta = std::acos(std::clamp(dir.z(), -1.0f, 1.0f));
    float phi = std::atan2(dir.y(), dir.x());
    if (phi < 0.0f) phi += 2 * PI;

    float weight = std::sin(theta);
    coeffs.weight += weight;

    int idx = 0;
    for (int l = 0; l <= l_max_; ++l) {
      for (int m = -l; m <= l; ++m) {
        float ylm = SH(l, m, theta, phi);
        coeffs.raw_coeffs(0, idx) += color.x() * ylm * weight;
        coeffs.raw_coeffs(1, idx) += color.y() * ylm * weight;
        coeffs.raw_coeffs(2, idx) += color.z() * ylm * weight;
        ++idx;
      }
    }
  }
}

void EllipsoidHarmonics::finalizeCoefficients(SHCoeffs& coeffs) const {
  coeffs.sh_coeffs = coeffs.raw_coeffs / coeffs.weight;
}

Eigen::Vector3f EllipsoidHarmonics::evaluateColorFromDirection(
    const SHCoeffs& coeffs, const Eigen::Vector3f& dir) const {
  Eigen::Vector3f n = dir.normalized();
  float x = n.x(), y = n.y(), z = n.z();
  float theta = std::acos(std::clamp(z, -1.0f, 1.0f));
  float phi = std::atan2(y, x);
  if (phi < 0.0f) phi += 2.0f * PI;

  Eigen::VectorXf Y(n_coeffs_);
  int idx = 0;
  for (int l = 0; l <= l_max_; ++l)
    for (int m = -l; m <= l; ++m) Y(idx++) = SH(l, m, theta, phi);

  Eigen::Vector3f color;
  for (int c = 0; c < 3; ++c) color[c] = coeffs.sh_coeffs.row(c).dot(Y);

  return color;
}

Eigen::Vector3f EllipsoidHarmonics::ellipsoidPointFromDir(
    const Eigen::Vector3f& dir, const Eigen::Vector3f& scale) const {
  Eigen::Vector3f n = dir.normalized();
  return Eigen::Vector3f(scale.x() * n.x(), scale.y() * n.y(),
                         scale.z() * n.z());
}

Eigen::Vector3f EllipsoidHarmonics::dirFromEllipsoidPoint(
    const Eigen::Vector3f& point, const Eigen::Vector3f& scale) const {
  float x = point.x() / scale.x();
  float y = point.y() / scale.y();
  float z = point.z() / scale.z();

  Eigen::Vector3f dir(x, y, z);
  return dir.normalized();
}

Eigen::Vector3f EllipsoidHarmonics::findDirectionMatchingColor(
    const SHCoeffs& coeffs, const Eigen::Vector3f& target_color,
    const Eigen::Vector3f& initial_dir, int max_iters, float epsilon) const {
  auto sph = cartesianToSpherical(initial_dir);
  dual2nd theta = sph[0];
  dual2nd phi = sph[1];

  for (int iter = 0; iter < max_iters; ++iter) {
    auto loss_fn = [&](const auto& vars) {
      dual2nd t = vars(0), p = vars(1);
      Eigen::VectorX<dual2nd> Y(n_coeffs_);

      int idx = 0;
      for (int l = 0; l <= l_max_; ++l)
        for (int m = -l; m <= l; ++m) Y(idx++) = SH(l, m, val(t), val(p));

      dual2nd loss = 0.0;
      for (int c = 0; c < 3; ++c) {
        auto col = coeffs.sh_coeffs.row(c).cast<dual2nd>().dot(Y);
        loss += pow(col - target_color[c], 2);
      }
      return loss;
    };

    Eigen::Vector2<dual2nd> vars(theta, phi);
    dual2nd loss;
    Eigen::Vector2d grad;
    Eigen::Matrix2d hess;

    hessian(loss_fn, wrt(vars), at(vars), loss, grad, hess);

    Eigen::Vector2d delta;
    bool failed = false;
    try {
      delta = -hess.ldlt().solve(grad);
      if (delta.norm() < 1e-8) failed = true;
    } catch (...) {
      failed = true;
    }

    if (failed) {
      delta = -grad.normalized() * 0.01;
    }

    theta = theta + delta(0);
    phi = phi + delta(1);

    theta = std::clamp(val(theta), 0.001, PI - 0.001);
    phi = std::fmod(val(phi), 2 * PI);
    if (phi < 0.0) phi += 2 * PI;

    if (grad.norm() < epsilon || delta.norm() < epsilon) break;
  }

  float th = val(theta);
  float ph = val(phi);
  return Eigen::Vector3f(std::sin(th) * std::cos(ph),
                         std::sin(th) * std::sin(ph), std::cos(th));
}

Eigen::Vector2f EllipsoidHarmonics::cartesianToSpherical(
    const Eigen::Vector3f& dir) const {
  Eigen::Vector3f n = dir.normalized();
  float theta =
      std::acos(std::clamp(n.z(), -1.0f, 1.0f));  // polar angle [0, π]
  float phi = std::atan2(n.y(), n.x());           // azimuthal angle [-π, π]

  if (phi < 0.0f) phi += 2 * PI;  // convert to [0, 2π)

  return Eigen::Vector2f(theta, phi);
}