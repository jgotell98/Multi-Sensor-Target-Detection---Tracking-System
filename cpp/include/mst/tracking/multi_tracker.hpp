#pragma once

#include <cstdint>
#include <variant>
#include <vector>

#include "mst/processing/signal_processing.hpp"
#include "mst/tracking/ekf.hpp"
#include "mst/tracking/kalman.hpp"
#include "mst/tracking/particle_filter.hpp"

namespace mst {

enum class TrackerKind { KF, EKF, PF };

struct TrackerConfig final {
  TrackerKind kind = TrackerKind::EKF;
  double dt_s = 0.05;
  double association_gate_m = 10.0;
  double association_gate_bearing_rad = 0.2;
  double track_timeout_s = 1.0;
  int max_tracks = 32;
  KalmanConfig kf{};
  EkfConfig ekf{};
  ParticleConfig pf{};
};

class MultiTracker final {
 public:
  explicit MultiTracker(TrackerConfig cfg, std::uint32_t seed);

  void step(double t_s,
            const std::vector<RadarDetection>& radar,
            const std::vector<AcousticDetection>& acoustic);

  std::vector<TrackEstimate> tracks() const;

 private:
  struct TrackKF final {
    std::int32_t id = 0;
    double age_s = 0.0;
    double last_update_s = 0.0;
    KalmanFilterCV f;
    explicit TrackKF(std::int32_t tid, KalmanConfig cfg) : id(tid), f(cfg) {}
  };
  struct TrackEKF final {
    std::int32_t id = 0;
    double age_s = 0.0;
    double last_update_s = 0.0;
    ExtendedKalmanFilterCV f;
    explicit TrackEKF(std::int32_t tid, EkfConfig cfg) : id(tid), f(cfg) {}
  };
  struct TrackPF final {
    std::int32_t id = 0;
    double age_s = 0.0;
    double last_update_s = 0.0;
    ParticleFilterCV f;
    explicit TrackPF(std::int32_t tid, ParticleConfig cfg, std::uint32_t seed) : id(tid), f(cfg, seed) {}
  };

  using Track = std::variant<TrackKF, TrackEKF, TrackPF>;

  TrackerConfig cfg_{};
  std::uint32_t seed_ = 1;
  std::int32_t next_id_ = 1;
  std::vector<Track> tracks_;

  void predict_all();
  void prune(double t_s);
  void spawn_from_radar(double t_s, const RadarDetection& d);
  void spawn_from_acoustic(double t_s, const AcousticDetection& d);
};

}  // namespace mst

