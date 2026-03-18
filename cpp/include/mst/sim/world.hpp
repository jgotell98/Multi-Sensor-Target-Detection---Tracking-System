#pragma once

#include <cstdint>
#include <random>
#include <vector>

#include "mst/types.hpp"

namespace mst {

struct WorldConfig final {
  int objects = 3;
  double arena_half_extent_m = 100.0;
  double min_speed_mps = 1.0;
  double max_speed_mps = 8.0;
  double process_noise_accel_std_mps2 = 0.3;
};

class World final {
 public:
  explicit World(WorldConfig cfg, std::uint32_t seed);

  const std::vector<TruthObject>& objects() const { return objects_; }
  void step(double dt_s);

 private:
  WorldConfig cfg_{};
  std::mt19937 rng_;
  std::normal_distribution<double> accel_noise_{0.0, 1.0};
  std::vector<TruthObject> objects_;

  void bounce_if_needed(TruthObject& o);
};

}  // namespace mst

