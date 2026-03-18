#include "mst/benchmark/metrics.hpp"

#include <algorithm>
#include <cmath>

namespace mst {
namespace {

static double percentile(std::vector<double> v, double p) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const double idx = (p / 100.0) * (static_cast<double>(v.size() - 1));
  const std::size_t i0 = static_cast<std::size_t>(std::floor(idx));
  const std::size_t i1 = std::min(v.size() - 1, i0 + 1);
  const double t = idx - static_cast<double>(i0);
  return (1.0 - t) * v[i0] + t * v[i1];
}

static double rmse_pos(const std::vector<std::vector<TruthObject>>& truth_by_frame,
                       const std::vector<std::vector<TrackEstimate>>& tracks_by_frame) {
  double sse = 0.0;
  std::size_t n = 0;

  const std::size_t frames = std::min(truth_by_frame.size(), tracks_by_frame.size());
  for (std::size_t k = 0; k < frames; ++k) {
    const auto& truth = truth_by_frame[k];
    const auto& tracks = tracks_by_frame[k];
    if (truth.empty() || tracks.empty()) continue;
    for (const auto& o : truth) {
      double best2 = 1e18;
      for (const auto& t : tracks) {
        const double dx = t.x - o.pos.x;
        const double dy = t.y - o.pos.y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < best2) best2 = d2;
      }
      sse += best2;
      ++n;
    }
  }

  if (n == 0) return 0.0;
  return std::sqrt(sse / static_cast<double>(n));
}

}  // namespace

BenchStats compute_bench(const std::vector<double>& frame_ms,
                         const std::vector<std::vector<TruthObject>>& truth_by_frame,
                         const std::vector<std::vector<TrackEstimate>>& tracks_by_frame) {
  BenchStats s;
  s.frames = frame_ms.size();
  if (!frame_ms.empty()) {
    double sum = 0.0;
    for (double v : frame_ms) sum += v;
    s.avg_frame_ms = sum / static_cast<double>(frame_ms.size());
    s.p95_frame_ms = percentile(frame_ms, 95.0);
  }
  s.rmse_pos_m = rmse_pos(truth_by_frame, tracks_by_frame);
  return s;
}

}  // namespace mst

