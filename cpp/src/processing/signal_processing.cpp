#include "mst/processing/signal_processing.hpp"

#include <cmath>
#include <vector>

namespace mst {
namespace {

struct Pt final {
  double x = 0.0;
  double y = 0.0;
  double snr = 0.0;
};

static double dist2(const Pt& a, const Pt& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

}  // namespace

std::vector<RadarDetection> SignalProcessor::detect_radar(const std::vector<RadarHit>& hits) const {
  std::vector<Pt> pts;
  std::vector<RadarHit> keep;
  keep.reserve(hits.size());
  for (const auto& h : hits) {
    if (h.snr < cfg_.snr_threshold) continue;
    keep.push_back(h);
    Pt p;
    p.x = h.range_m * std::cos(h.bearing_rad);
    p.y = h.range_m * std::sin(h.bearing_rad);
    p.snr = h.snr;
    pts.push_back(p);
  }

  const double rad2 = cfg_.cluster_radius_m * cfg_.cluster_radius_m;
  std::vector<char> used(pts.size(), 0);
  std::vector<RadarDetection> out;

  for (std::size_t i = 0; i < pts.size(); ++i) {
    if (used[i]) continue;
    std::vector<std::size_t> cluster;
    cluster.push_back(i);
    used[i] = 1;

    for (std::size_t idx = 0; idx < cluster.size(); ++idx) {
      const std::size_t j = cluster[idx];
      for (std::size_t k = 0; k < pts.size(); ++k) {
        if (used[k]) continue;
        if (dist2(pts[j], pts[k]) <= rad2) {
          used[k] = 1;
          cluster.push_back(k);
        }
      }
    }

    if (static_cast<int>(cluster.size()) < cfg_.min_cluster_points) continue;
    double sx = 0.0, sy = 0.0, ssnr = 0.0;
    for (auto idx : cluster) {
      sx += pts[idx].x;
      sy += pts[idx].y;
      ssnr += pts[idx].snr;
    }
    const double cx = sx / cluster.size();
    const double cy = sy / cluster.size();
    RadarDetection d;
    d.range_m = std::sqrt(cx * cx + cy * cy);
    d.bearing_rad = std::atan2(cy, cx);
    d.snr = ssnr / cluster.size();
    out.push_back(d);
  }

  return out;
}

std::vector<AcousticDetection> SignalProcessor::detect_acoustic(const std::vector<AcousticHit>& hits) const {
  std::vector<double> bearings;
  std::vector<double> snr;
  bearings.reserve(hits.size());
  snr.reserve(hits.size());
  for (const auto& h : hits) {
    if (h.snr < cfg_.snr_threshold) continue;
    bearings.push_back(h.bearing_rad);
    snr.push_back(h.snr);
  }

  if (bearings.empty()) return {};

  std::vector<char> used(bearings.size(), 0);
  std::vector<AcousticDetection> out;

  const double max_db = 0.08;
  for (std::size_t i = 0; i < bearings.size(); ++i) {
    if (used[i]) continue;
    std::vector<std::size_t> cluster;
    cluster.push_back(i);
    used[i] = 1;
    for (std::size_t idx = 0; idx < cluster.size(); ++idx) {
      const std::size_t j = cluster[idx];
      for (std::size_t k = 0; k < bearings.size(); ++k) {
        if (used[k]) continue;
        if (std::abs(bearings[j] - bearings[k]) <= max_db) {
          used[k] = 1;
          cluster.push_back(k);
        }
      }
    }
    if (static_cast<int>(cluster.size()) < cfg_.min_cluster_points) continue;
    double sb = 0.0, ssnr = 0.0;
    for (auto idx : cluster) {
      sb += bearings[idx];
      ssnr += snr[idx];
    }
    AcousticDetection d;
    d.bearing_rad = sb / cluster.size();
    d.snr = ssnr / cluster.size();
    out.push_back(d);
  }

  return out;
}

}  // namespace mst

