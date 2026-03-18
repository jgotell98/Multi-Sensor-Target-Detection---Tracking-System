#pragma once

#include <cstddef>
#include <vector>

#include "mst/types.hpp"

namespace mst {

struct BenchStats final {
  double avg_frame_ms = 0.0;
  double p95_frame_ms = 0.0;
  double rmse_pos_m = 0.0;
  std::size_t frames = 0;
};

BenchStats compute_bench(const std::vector<double>& frame_ms,
                         const std::vector<std::vector<TruthObject>>& truth_by_frame,
                         const std::vector<std::vector<TrackEstimate>>& tracks_by_frame);

}  // namespace mst

