#include <ellipsoid_harmonics.h>

#include <algorithm>
#include <cmath>
#include <iostream>

constexpr float PI = 3.14159265359f;

EllipsoidHarmonics::EllipsoidHarmonics(int l_max, bool use_gpu)
    : l_max_(l_max),
      use_gpu_(use_gpu),
      n_coeffs_((l_max + 1) * (l_max + 1)),
      a_(1.0f),
      b_(1.0f),
      c_(1.0f) {}

void EllipsoidHarmonics::setEllipsoidAxes(float a, float b, float c) {
  a_ = a;
  b_ = b;
  c_ = c;
}

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
    SHAccumulation& accum) const {
  for (size_t i = 0; i < directions.size(); ++i) {
    const auto& dir = directions[i].normalized();
    const auto& color = colors[i];

    float theta = std::acos(std::clamp(dir.z(), -1.0f, 1.0f));
    float phi = std::atan2(dir.y(), dir.x());
    if (phi < 0.0f) phi += 2 * PI;

    float weight = std::sin(theta);
    accum.weight += weight;

    int idx = 0;
    for (int l = 0; l <= l_max_; ++l) {
      for (int m = -l; m <= l; ++m) {
        float ylm = SH(l, m, theta, phi);
        accum.coeffs[0](idx) += color.x() * ylm * weight;
        accum.coeffs[1](idx) += color.y() * ylm * weight;
        accum.coeffs[2](idx) += color.z() * ylm * weight;
        ++idx;
      }
    }
  }
}

void EllipsoidHarmonics::finalizeCoefficients(const SHAccumulation& accum,
                                              SHCoeffs& out_coeffs) const {
  out_coeffs.resize(3);
  for (int i = 0; i < 3; ++i) {
    out_coeffs[i] = accum.coeffs[i] / accum.weight;
  }
}

void EllipsoidHarmonics::computeCoefficients(
    const std::vector<Vec3f>& directions, const std::vector<Vec3f>& colors,
    SHCoeffs& out_coeffs) {
  if (use_gpu_) {
    computeOnGPU(directions, colors, out_coeffs);
    return;
  }

  SHAccumulation accum;
  accum.weight = 0.0f;
  accum.coeffs.resize(3);
  for (auto& c : accum.coeffs) {
    c = Eigen::VectorXf::Zero(n_coeffs_);
  }

  accumulateCoefficients(directions, colors, accum);
  finalizeCoefficients(accum, out_coeffs);
}

void EllipsoidHarmonics::computeOnGPU(const std::vector<Vec3f>& directions,
                                      const std::vector<Vec3f>& colors,
                                      SHCoeffs& out_coeffs) {
  std::cerr << "GPU acceleration not yet implemented.\n";
  std::exit(1);
}

Eigen::Vector3f EllipsoidHarmonics::evaluateColorFromDirection(
    const SHCoeffs& sh_coeffs, const Eigen::Vector3f& dir) const {
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
  for (int c = 0; c < 3; ++c) color[c] = sh_coeffs[c].dot(Y);

  return color;
}

Eigen::Vector3f EllipsoidHarmonics::ellipsoidPointFromDir(
    const Eigen::Vector3f& dir) const {
  Eigen::Vector3f n = dir.normalized();
  return Eigen::Vector3f(a_ * n.x(), b_ * n.y(), c_ * n.z());
}

Eigen::Vector3f EllipsoidHarmonics::evaluateColorOnEllipsoidFromDir(
    const Eigen::Vector3f& dir, const SHCoeffs& coeffs) const {
  return evaluateColorFromDirection(coeffs, dir);
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
      VectorX<dual2nd> Y(n_coeffs_);

      int idx = 0;
      for (int l = 0; l <= l_max_; ++l)
        for (int m = -l; m <= l; ++m) Y(idx++) = SH(l, m, t, p);

      dual2nd loss = 0.0;
      for (int c = 0; c < 3; ++c) {
        auto col = coeffs[c].cast<dual2nd>().dot(Y);
        loss += pow(col - target_color[c], 2);
      }
      return loss;
    };

    Eigen::Vector2<dual2nd> vars(theta, phi);
    dual2nd loss;
    Eigen::Vector2d grad;
    Eigen::Matrix2d hess;

    loss = hessian(loss_fn, wrt(vars), at(vars), grad, hess);

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

    theta = std::clamp(theta, 0.001, PI - 0.001);
    phi = std::fmod(phi, 2 * PI);
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