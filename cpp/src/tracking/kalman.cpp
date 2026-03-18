#include "mst/tracking/kalman.hpp"

#include <cmath>

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

KalmanFilterCV::KalmanFilterCV(KalmanConfig cfg) : cfg_(cfg) {}

void KalmanFilterCV::init(double x, double y, double vx, double vy) {
  x_(0, 0) = x;
  x_(1, 0) = y;
  x_(2, 0) = vx;
  x_(3, 0) = vy;
  P_ = Matrix<4, 4>::identity();
  P_(0, 0) = 25.0;
  P_(1, 1) = 25.0;
  P_(2, 2) = 25.0;
  P_(3, 3) = 25.0;
  inited_ = true;
}

void KalmanFilterCV::predict(double dt_s) {
  if (!inited_) return;
  const Matrix<4, 4> F = make_F(dt_s);
  const Matrix<4, 4> Q = make_Q(dt_s, cfg_.process_accel_std_mps2);
  x_ = F * x_;
  P_ = F * P_ * transpose(F) + Q;
}

void KalmanFilterCV::update_cartesian(double meas_x, double meas_y) {
  if (!inited_) {
    init(meas_x, meas_y, 0.0, 0.0);
    return;
  }
  Matrix<2, 4> H;
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;

  Matrix<2, 1> z;
  z(0, 0) = meas_x;
  z(1, 0) = meas_y;

  Matrix<2, 2> R;
  const double r = cfg_.meas_pos_std_m * cfg_.meas_pos_std_m;
  R(0, 0) = r;
  R(1, 1) = r;

  const Matrix<2, 1> y = z - (H * x_);
  const Matrix<2, 2> S = H * P_ * transpose(H) + R;
  const Matrix<4, 2> K = P_ * transpose(H) * inverse(S);
  x_ = x_ + K * y;
  const Matrix<4, 4> I = Matrix<4, 4>::identity();
  P_ = (I - K * H) * P_;
}

TrackClass classify_from_state(const Matrix<4, 1>& x) {
  const double v = std::sqrt(x(2, 0) * x(2, 0) + x(3, 0) * x(3, 0));
  if (v < 0.6) return TrackClass::Stationary;
  if (v < 4.0) return TrackClass::Slow;
  return TrackClass::Fast;
}

}  // namespace mst

