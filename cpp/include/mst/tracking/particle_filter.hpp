#pragma once

#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

#include "mst/math/util.hpp"
#include "mst/types.hpp"

namespace mst {

struct ParticleConfig final {
  std::size_t particles = 800;
  double process_accel_std_mps2 = 1.2;
  double meas_bearing_std_rad = 0.04;
  double init_pos_std_m = 30.0;
  double init_vel_std_mps = 4.0;
};

struct Particle final {
  double x = 0.0;
  double y = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double w = 1.0;
};

class ParticleFilterCV final {
 public:
  ParticleFilterCV(ParticleConfig cfg, std::uint32_t seed);

  void init();
  void predict(double dt_s);
  void update_bearing(double bearing_rad);
  void resample_if_needed();
  TrackEstimate estimate(std::int32_t id, double t_s, double age_s, double last_update_s) const;

 private:
  ParticleConfig cfg_{};
  std::mt19937 rng_;
  std::normal_distribution<double> norm_{0.0, 1.0};
  std::vector<Particle> p_;

  double effective_n() const;
  void systematic_resample();
};

}  // namespace mst

