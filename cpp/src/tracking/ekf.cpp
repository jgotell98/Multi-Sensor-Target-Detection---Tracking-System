#include "mst/tracking/ekf.hpp"

#include <algorithm>
#include <cmath>

#include "mst/math/util.hpp"

namespace mst {
namespace {

static Matrix<4, 4> make_F(double dt) {
  Matrix<4, 4> F = Matrix<4, 4>::identity();
  F(0, 2) = dt;
  F(1, 3) = dt;
  return F;
}

static Matrix<4, 4> make_Q(double dt, double accel_std) {
  const double q = accel_std * accel_std;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;
  Matrix<4, 4> Q;
  Q(0, 0) = 0.25 * dt4 * q;
  Q(0, 2) = 0.5 * dt3 * q;
  Q(1, 1) = 0.25 * dt4 * q;
  Q(1, 3) = 0.5 * dt3 * q;
  Q(2, 0) = 0.5 * dt3 * q;
  Q(2, 2) = dt2 * q;
  Q(3, 1) = 0.5 * dt3 * q;
  Q(3, 3) = dt2 * q;
  return Q;
}

}  // namespace

ExtendedKalmanFilterCV::ExtendedKalmanFilterCV(EkfConfig cfg) : cfg_(cfg) {}

void ExtendedKalmanFilterCV::init(double x, double y, double vx, double vy) {
  x_(0, 0) = x;
  x_(1, 0) = y;
  x_(2, 0) = vx;
  x_(3, 0) = vy;
  P_ = Matrix<4, 4>::identity();
  P_(0, 0) = 36.0;
  P_(1, 1) = 36.0;
  P_(2, 2) = 36.0;
  P_(3, 3) = 36.0;
  inited_ = true;
}

void ExtendedKalmanFilterCV::predict(double dt_s) {
  if (!inited_) return;
  const Matrix<4, 4> F = make_F(dt_s);
  const Matrix<4, 4> Q = make_Q(dt_s, cfg_.process_accel_std_mps2);
  x_ = F * x_;
  P_ = F * P_ * transpose(F) + Q;
}

void ExtendedKalmanFilterCV::update_radar(double range_m, double bearing_rad) {
  if (!inited_) {
    const double x = range_m * std::cos(bearing_rad);
    const double y = range_m * std::sin(bearing_rad);
    init(x, y, 0.0, 0.0);
    return;
  }

  const double px = x_(0, 0);
  const double py = x_(1, 0);
  const double r2 = std::max(1e-9, px * px + py * py);
  const double r = std::sqrt(r2);

  Matrix<2, 1> z;
  z(0, 0) = range_m;
  z(1, 0) = bearing_rad;

  Matrix<2, 1> h;
  h(0, 0) = r;
  h(1, 0) = std::atan2(py, px);

  Matrix<2, 4> Hj;
  Hj(0, 0) = px / r;
  Hj(0, 1) = py / r;
  Hj(1, 0) = -py / r2;
  Hj(1, 1) = px / r2;

  Matrix<2, 2> R;
  R(0, 0) = cfg_.meas_range_std_m * cfg_.meas_range_std_m;
  R(1, 1) = cfg_.meas_bearing_std_rad * cfg_.meas_bearing_std_rad;

  Matrix<2, 1> y = z - h;
  y(1, 0) = wrap_angle_rad(y(1, 0));

  const Matrix<2, 2> S = Hj * P_ * transpose(Hj) + R;
  const Matrix<4, 2> K = P_ * transpose(Hj) * inverse(S);
  x_ = x_ + K * y;
  const Matrix<4, 4> I = Matrix<4, 4>::identity();
  P_ = (I - K * Hj) * P_;
}

void ExtendedKalmanFilterCV::update_bearing_only(double bearing_rad, double bearing_std_rad) {
  if (!inited_) return;

  const double px = x_(0, 0);
  const double py = x_(1, 0);
  const double r2 = std::max(1e-9, px * px + py * py);

  const double pred = std::atan2(py, px);
  Matrix<1, 1> z;
  z(0, 0) = bearing_rad;
  Matrix<1, 1> h;
  h(0, 0) = pred;

  Matrix<1, 4> H;
  H(0, 0) = -py / r2;
  H(0, 1) = px / r2;
  H(0, 2) = 0.0;
  H(0, 3) = 0.0;

  Matrix<1, 1> R;
  R(0, 0) = bearing_std_rad * bearing_std_rad;

  Matrix<1, 1> y = z - h;
  y(0, 0) = wrap_angle_rad(y(0, 0));

  const Matrix<1, 1> S = H * P_ * transpose(H) + R;
  const Matrix<4, 1> K = P_ * transpose(H) * inverse(S);
  x_ = x_ + K * y;
  const Matrix<4, 4> I = Matrix<4, 4>::identity();
  P_ = (I - K * H) * P_;
}

}  // namespace mst
