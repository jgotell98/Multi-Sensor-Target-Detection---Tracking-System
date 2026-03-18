#pragma once

#include <cstdint>
#include <random>
#include <vector>

#include "mst/types.hpp"

namespace mst {

struct RadarConfig final {
  double max_range_m = 250.0;
  double range_noise_std_m = 1.5;
  double bearing_noise_std_rad = 0.01;
  int returns_per_object = 8;
  int clutter_returns = 30;
  double snr_object_mean = 12.0;
  double snr_object_std = 2.0;
  double snr_clutter_mean = 3.0;
  double snr_clutter_std = 1.0;
};

struct AcousticConfig final {
  double bearing_noise_std_rad = 0.03;
  int returns_per_object = 6;
  int clutter_returns = 25;
  double snr_object_mean = 8.0;
  double snr_object_std = 2.0;
  double snr_clutter_mean = 2.0;
  double snr_clutter_std = 1.0;
};

class SensorSimulator final {
 public:
  SensorSimulator(RadarConfig radar, AcousticConfig acoustic, std::uint32_t seed);

  std::vector<RadarHit> simulate_radar(const std::vector<TruthObject>& truth);
  std::vector<AcousticHit> simulate_acoustic(const std::vector<TruthObject>& truth);

 private:
  RadarConfig radar_{};
  AcousticConfig acoustic_{};
  std::mt19937 rng_;
  std::normal_distribution<double> norm_{0.0, 1.0};
  std::uniform_real_distribution<double> uni01_{0.0, 1.0};
};

}  // namespace mst

