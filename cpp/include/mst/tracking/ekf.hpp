#pragma once

#include "mst/math/matrix.hpp"
#include "mst/types.hpp"

namespace mst {

struct EkfConfig final {
  double process_accel_std_mps2 = 0.8;
  double meas_range_std_m = 1.8;
  double meas_bearing_std_rad = 0.012;
};

class ExtendedKalmanFilterCV final {
 public:
  explicit ExtendedKalmanFilterCV(EkfConfig cfg);

  void init(double x, double y, double vx, double vy);
  void predict(double dt_s);
  void update_radar(double range_m, double bearing_rad);
  void update_bearing_only(double bearing_rad, double bearing_std_rad);

  Matrix<4, 1> state() const { return x_; }
  Matrix<4, 4> cov() const { return P_; }

 private:
  EkfConfig cfg_{};
  Matrix<4, 1> x_{};
  Matrix<4, 4> P_{};
  bool inited_ = false;
};

}  // namespace mst
