#pragma once

#include "mst/math/matrix.hpp"
#include "mst/types.hpp"

namespace mst {

struct KalmanConfig final {
  double process_accel_std_mps2 = 0.8;
  double meas_pos_std_m = 2.5;
};

class KalmanFilterCV final {
 public:
  explicit KalmanFilterCV(KalmanConfig cfg);

  void init(double x, double y, double vx, double vy);
  void predict(double dt_s);
  void update_cartesian(double meas_x, double meas_y);

  Matrix<4, 1> state() const { return x_; }
  Matrix<4, 4> cov() const { return P_; }

 private:
  KalmanConfig cfg_{};
  Matrix<4, 1> x_{};
  Matrix<4, 4> P_{};
  bool inited_ = false;
};

TrackClass classify_from_state(const Matrix<4, 1>& x);

}  // namespace mst

