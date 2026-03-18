#include "mst/tracking/multi_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <type_traits>

#include "mst/math/util.hpp"

namespace mst {
namespace {

static double dist2(double ax, double ay, double bx, double by) {
  const double dx = ax - bx;
  const double dy = ay - by;
  return dx * dx + dy * dy;
}

}  // namespace

MultiTracker::MultiTracker(TrackerConfig cfg, std::uint32_t seed) : cfg_(cfg), seed_(seed) {}

void MultiTracker::predict_all() {
  for (auto& tv : tracks_) {
    std::visit(
        [&](auto& t) {
          t.age_s += cfg_.dt_s;
          if constexpr (std::is_same_v<std::decay_t<decltype(t)>, TrackKF>) t.f.predict(cfg_.dt_s);
          if constexpr (std::is_same_v<std::decay_t<decltype(t)>, TrackEKF>) t.f.predict(cfg_.dt_s);
          if constexpr (std::is_same_v<std::decay_t<decltype(t)>, TrackPF>) t.f.predict(cfg_.dt_s);
        },
        tv);
  }
}

void MultiTracker::prune(double t_s) {
  tracks_.erase(std::remove_if(tracks_.begin(),
                               tracks_.end(),
                               [&](const Track& tv) {
                                 return std::visit(
                                     [&](const auto& t) { return (t_s - t.last_update_s) > cfg_.track_timeout_s; }, tv);
                               }),
                tracks_.end());
}

void MultiTracker::spawn_from_radar(double t_s, const RadarDetection& d) {
  if (static_cast<int>(tracks_.size()) >= cfg_.max_tracks) return;
  const std::int32_t id = next_id_++;
  const double x = d.range_m * std::cos(d.bearing_rad);
  const double y = d.range_m * std::sin(d.bearing_rad);

  if (cfg_.kind == TrackerKind::KF) {
    TrackKF t(id, cfg_.kf);
    t.last_update_s = t_s;
    t.f.init(x, y, 0.0, 0.0);
    tracks_.push_back(std::move(t));
  } else if (cfg_.kind == TrackerKind::EKF) {
    TrackEKF t(id, cfg_.ekf);
    t.last_update_s = t_s;
    t.f.init(x, y, 0.0, 0.0);
    tracks_.push_back(std::move(t));
  }
}

void MultiTracker::spawn_from_acoustic(double t_s, const AcousticDetection& d) {
  if (static_cast<int>(tracks_.size()) >= cfg_.max_tracks) return;
  if (cfg_.kind != TrackerKind::PF) return;
  const std::int32_t id = next_id_++;
  TrackPF t(id, cfg_.pf, seed_ + static_cast<std::uint32_t>(id));
  t.last_update_s = t_s;
  t.f.init();
  t.f.update_bearing(d.bearing_rad);
  t.f.resample_if_needed();
  tracks_.push_back(std::move(t));
}

void MultiTracker::step(double t_s,
                        const std::vector<RadarDetection>& radar,
                        const std::vector<AcousticDetection>& acoustic) {
  predict_all();

  if (cfg_.kind == TrackerKind::KF || cfg_.kind == TrackerKind::EKF) {
    std::vector<char> used_det(radar.size(), 0);

    for (auto& tv : tracks_) {
      std::visit(
          [&](auto& t) {
            if constexpr (std::is_same_v<std::decay_t<decltype(t)>, TrackKF> ||
                          std::is_same_v<std::decay_t<decltype(t)>, TrackEKF>) {
              const auto x = t.f.state();
              const double px = x(0, 0);
              const double py = x(1, 0);

              int best = -1;
              double best_d2 = cfg_.association_gate_m * cfg_.association_gate_m;
              for (std::size_t i = 0; i < radar.size(); ++i) {
                if (used_det[i]) continue;
                const double mx = radar[i].range_m * std::cos(radar[i].bearing_rad);
                const double my = radar[i].range_m * std::sin(radar[i].bearing_rad);
                const double d2 = dist2(px, py, mx, my);
                if (d2 < best_d2) {
                  best_d2 = d2;
                  best = static_cast<int>(i);
                }
              }

              if (best >= 0) {
                used_det[static_cast<std::size_t>(best)] = 1;
                t.last_update_s = t_s;
                if constexpr (std::is_same_v<std::decay_t<decltype(t)>, TrackKF>) {
                  const double mx = radar[best].range_m * std::cos(radar[best].bearing_rad);
                  const double my = radar[best].range_m * std::sin(radar[best].bearing_rad);
                  t.f.update_cartesian(mx, my);
                } else {
                  t.f.update_radar(radar[best].range_m, radar[best].bearing_rad);
                }
              }
            }
          },
          tv);
    }

    for (std::size_t i = 0; i < radar.size(); ++i) {
      if (!used_det[i]) spawn_from_radar(t_s, radar[i]);
    }

    // Optional multi-sensor fusion: acoustic bearing-only updates for EKF tracks.
    if (cfg_.kind == TrackerKind::EKF && !acoustic.empty()) {
      std::vector<char> used_ac(acoustic.size(), 0);
      for (auto& tv : tracks_) {
        std::visit(
            [&](auto& t) {
              if constexpr (std::is_same_v<std::decay_t<decltype(t)>, TrackEKF>) {
                const auto x = t.f.state();
                const double pred_b = std::atan2(x(1, 0), x(0, 0));
                int best = -1;
                double best_db = cfg_.association_gate_bearing_rad;
                for (std::size_t i = 0; i < acoustic.size(); ++i) {
                  if (used_ac[i]) continue;
                  const double db = std::abs(wrap_angle_rad(acoustic[i].bearing_rad - pred_b));
                  if (db < best_db) {
                    best_db = db;
                    best = static_cast<int>(i);
                  }
                }
                if (best >= 0) {
                  used_ac[static_cast<std::size_t>(best)] = 1;
                  t.last_update_s = t_s;
                  t.f.update_bearing_only(acoustic[best].bearing_rad, cfg_.ekf.meas_bearing_std_rad * 2.5);
                }
              }
            },
            tv);
      }
    }
  } else if (cfg_.kind == TrackerKind::PF) {
    std::vector<char> used(acoustic.size(), 0);

    for (auto& tv : tracks_) {
      std::visit(
          [&](auto& t) {
            if constexpr (std::is_same_v<std::decay_t<decltype(t)>, TrackPF>) {
              const TrackEstimate e = t.f.estimate(t.id, t_s, t.age_s, t.last_update_s);
              const double pred_b = std::atan2(e.y, e.x);

              int best = -1;
              double best_db = cfg_.association_gate_bearing_rad;
              for (std::size_t i = 0; i < acoustic.size(); ++i) {
                if (used[i]) continue;
                const double db = std::abs(wrap_angle_rad(acoustic[i].bearing_rad - pred_b));
                if (db < best_db) {
                  best_db = db;
                  best = static_cast<int>(i);
                }
              }
              if (best >= 0) {
                used[static_cast<std::size_t>(best)] = 1;
                t.last_update_s = t_s;
                t.f.update_bearing(acoustic[best].bearing_rad);
                t.f.resample_if_needed();
              }
            }
          },
          tv);
    }

    for (std::size_t i = 0; i < acoustic.size(); ++i) {
      if (!used[i]) spawn_from_acoustic(t_s, acoustic[i]);
    }
  }

  prune(t_s);
}

std::vector<TrackEstimate> MultiTracker::tracks() const {
  std::vector<TrackEstimate> out;
  out.reserve(tracks_.size());
  for (const auto& tv : tracks_) {
    std::visit(
        [&](const auto& t) {
          if constexpr (std::is_same_v<std::decay_t<decltype(t)>, TrackKF> ||
                        std::is_same_v<std::decay_t<decltype(t)>, TrackEKF>) {
            const auto x = t.f.state();
            TrackEstimate e;
            e.id = t.id;
            e.x = x(0, 0);
            e.y = x(1, 0);
            e.vx = x(2, 0);
            e.vy = x(3, 0);
            e.age_s = t.age_s;
            e.last_update_s = t.last_update_s;
            e.cls = classify_from_state(x);
            out.push_back(e);
          } else if constexpr (std::is_same_v<std::decay_t<decltype(t)>, TrackPF>) {
            out.push_back(t.f.estimate(t.id, 0.0, t.age_s, t.last_update_s));
          }
        },
        tv);
  }
  return out;
}

}  // namespace mst
