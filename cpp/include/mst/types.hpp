#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace mst {

struct Vec2 final {
  double x = 0.0;
  double y = 0.0;
};

struct TruthObject final {
  std::int32_t id = 0;
  Vec2 pos{};
  Vec2 vel{};
};

enum class SensorKind { Radar, Acoustic };

struct RadarHit final {
  double range_m = 0.0;
  double bearing_rad = 0.0;
  double snr = 0.0;
};

struct AcousticHit final {
  double bearing_rad = 0.0;
  double snr = 0.0;
};

struct RadarDetection final {
  double range_m = 0.0;
  double bearing_rad = 0.0;
  double snr = 0.0;
};

struct AcousticDetection final {
  double bearing_rad = 0.0;
  double snr = 0.0;
};

enum class TrackClass { Stationary, Slow, Fast };

inline std::string to_string(TrackClass c) {
  switch (c) {
    case TrackClass::Stationary:
      return "stationary";
    case TrackClass::Slow:
      return "slow";
    case TrackClass::Fast:
      return "fast";
  }
  return "unknown";
}

struct TrackEstimate final {
  std::int32_t id = 0;
  double x = 0.0;
  double y = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  TrackClass cls = TrackClass::Slow;
  double age_s = 0.0;
  double last_update_s = 0.0;
};

struct Frame final {
  double t_s = 0.0;
  std::vector<TruthObject> truth;
  std::vector<RadarHit> radar_hits;
  std::vector<AcousticHit> acoustic_hits;
};

}  // namespace mst

