#include <ellipsoid_harmonics.h>

EllipsoidHarmonics::EllipsoidHarmonics(int l_max)
    : l_max_(l_max), n_coeffs_((l_max + 1) * (l_max + 1)) {}

int EllipsoidHarmonics::getCoefficientCount() const { return n_coeffs_; }

float EllipsoidHarmonics::K(int l, int m) const {
  return std::sqrt((2 * l + 1) * std::tgamma(l - m + 1) /
                   (4 * M_PI * std::tgamma(l + m + 1)));
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

void EllipsoidHarmonics::computeCoefficients(const Vec3f& dir,
                                             const Vec3f& color,
                                             SHCoeffs& coeffs,
                                             int p_idx) const {
  auto sph = cartesianToSpherical(dir);
  float theta = sph[0];
  float phi = sph[1];

  coeffs.weights(p_idx) = std::sin(theta);

#pragma omp parallel for
  for (int l = 0; l < l_max_ + 1; l++) {
#pragma omp parallel for
    for (int m = -l; m < l + 1; m++) {
      int c_idx = l * l + (m + l);
      float ylm = SH(l, m, theta, phi);
      coeffs.r_coeffs(p_idx, c_idx) = color(0) * ylm * coeffs.weights(p_idx);
      coeffs.g_coeffs(p_idx, c_idx) = color(1) * ylm * coeffs.weights(p_idx);
      coeffs.b_coeffs(p_idx, c_idx) = color(2) * ylm * coeffs.weights(p_idx);
    }
  }
}

void EllipsoidHarmonics::finalizeCoefficients(SHCoeffs& coeffs,
                                              Eigen::MatrixXf& sh_mat) const {
  float total_weight = coeffs.weights.sum();
  sh_mat.row(0) = coeffs.r_coeffs.colwise().sum() / total_weight;
  sh_mat.row(1) = coeffs.g_coeffs.colwise().sum() / total_weight;
  sh_mat.row(2) = coeffs.b_coeffs.colwise().sum() / total_weight;
}

Vec3f EllipsoidHarmonics::evaluateColorFromDirection(
    const Eigen::MatrixXf& sh_mat, const Vec3f& dir) const {
  Vec3f n = dir.normalized();
  float x = n(0), y = n(1), z = n(2);
  float theta = std::acos(std::clamp(z, -1.0f, 1.0f));
  float phi = std::atan2(y, x);
  if (phi < 0.0f) phi += 2.0f * M_PI;

  Eigen::VectorXf Y(n_coeffs_);
  int idx = 0;
  for (int l = 0; l <= l_max_; ++l)
    for (int m = -l; m <= l; ++m) Y(idx++) = SH(l, m, theta, phi);

  Vec3f color;
  for (int c = 0; c < 3; ++c) color[c] = sh_mat.row(c).dot(Y);

  return color;
}

Vec3f EllipsoidHarmonics::ellipsoidPointFromDir(const Vec3f& dir,
                                                const Vec3f& scale,
                                                const Mat3f& rot) const {
  Vec3f n = dir.normalized();
  n = rot.transpose() * n;        // Apply rotation
  n = n.array() * scale.array();  // Scale to ellipsoid
  n = rot * n;                    // Apply rotation back
  return n;
}

Vec3f EllipsoidHarmonics::dirFromEllipsoidPoint(const Vec3f& point,
                                                const Vec3f& scale,
                                                const Mat3f& rot) const {
  Vec3f n = rot.transpose() * point;  // Apply inverse rotation
  n = n.array() / scale.array();      // Scale back to unit sphere
  n = rot * n;                        // Apply rotation back
  n.normalize();                      // Normalize to get direction
  return n;
}

Vec3f EllipsoidHarmonics::findDirectionMatchingColor(
    const Eigen::MatrixXf& sh_mat, const Vec3f& target_color,
    const Vec3f& initial_dir, int max_iters, float epsilon) const {
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
        auto col = sh_mat.row(c).cast<dual2nd>().dot(Y);
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

    theta = std::clamp(val(theta), 0.001, M_PI - 0.001);
    phi = std::fmod(val(phi), 2 * M_PI);
    if (phi < 0.0) phi += 2 * M_PI;

    if (grad.norm() < epsilon || delta.norm() < epsilon) break;
  }

  float th = val(theta);
  float ph = val(phi);
  return Vec3f(std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph),
               std::cos(th));
}

Eigen::Vector2f EllipsoidHarmonics::cartesianToSpherical(
    const Vec3f& dir) const {
  Vec3f n = dir.normalized();
  float theta = std::acos(std::clamp(n(2), -1.0f, 1.0f));  // polar angle [0, π]
  float phi = std::atan2(n(1), n(0));  // azimuthal angle [-π, π]

  if (phi < 0.0f) phi += 2 * M_PI;  // convert to [0, 2π)

  return Eigen::Vector2f(theta, phi);
}