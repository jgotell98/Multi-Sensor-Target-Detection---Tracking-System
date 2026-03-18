#include "mst/sim/sensors.hpp"

#include <cmath>

#include "mst/math/util.hpp"

namespace mst {

SensorSimulator::SensorSimulator(RadarConfig radar, AcousticConfig acoustic, std::uint32_t seed)
    : radar_(radar), acoustic_(acoustic), rng_(seed) {}

std::vector<RadarHit> SensorSimulator::simulate_radar(const std::vector<TruthObject>& truth) {
  std::vector<RadarHit> hits;
  hits.reserve(static_cast<std::size_t>(radar_.returns_per_object * static_cast<int>(truth.size()) + radar_.clutter_returns));

  for (const auto& o : truth) {
    const double true_r = std::sqrt(sqr(o.pos.x) + sqr(o.pos.y));
    const double true_b = std::atan2(o.pos.y, o.pos.x);
    if (true_r > radar_.max_range_m) continue;

    for (int k = 0; k < radar_.returns_per_object; ++k) {
      RadarHit h;
      h.range_m = true_r + radar_.range_noise_std_m * norm_(rng_);
      h.bearing_rad = wrap_angle_rad(true_b + radar_.bearing_noise_std_rad * norm_(rng_));
      h.snr = std::max(0.0, radar_.snr_object_mean + radar_.snr_object_std * norm_(rng_));
      hits.push_back(h);
    }
  }

  for (int k = 0; k < radar_.clutter_returns; ++k) {
    RadarHit h;
    h.range_m = radar_.max_range_m * uni01_(rng_);
    h.bearing_rad = wrap_angle_rad(-kPi + 2.0 * kPi * uni01_(rng_));
    h.snr = std::max(0.0, radar_.snr_clutter_mean + radar_.snr_clutter_std * norm_(rng_));
    hits.push_back(h);
  }
  return hits;
}

std::vector<AcousticHit> SensorSimulator::simulate_acoustic(const std::vector<TruthObject>& truth) {
  std::vector<AcousticHit> hits;
  hits.reserve(static_cast<std::size_t>(acoustic_.returns_per_object * static_cast<int>(truth.size()) + acoustic_.clutter_returns));

  for (const auto& o : truth) {
    const double true_b = std::atan2(o.pos.y, o.pos.x);
    for (int k = 0; k < acoustic_.returns_per_object; ++k) {
      AcousticHit h;
      h.bearing_rad = wrap_angle_rad(true_b + acoustic_.bearing_noise_std_rad * norm_(rng_));
      h.snr = std::max(0.0, acoustic_.snr_object_mean + acoustic_.snr_object_std * norm_(rng_));
      hits.push_back(h);
    }
  }

  for (int k = 0; k < acoustic_.clutter_returns; ++k) {
    AcousticHit h;
    h.bearing_rad = wrap_angle_rad(-kPi + 2.0 * kPi * uni01_(rng_));
    h.snr = std::max(0.0, acoustic_.snr_clutter_mean + acoustic_.snr_clutter_std * norm_(rng_));
    hits.push_back(h);
  }
  return hits;
}

}  // namespace mst
