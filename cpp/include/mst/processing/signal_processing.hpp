#pragma once

#include <vector>

#include "mst/types.hpp"

namespace mst {

struct SignalProcessingConfig final {
  double snr_threshold = 6.0;
  double cluster_radius_m = 3.5;
  int min_cluster_points = 3;
};

class SignalProcessor final {
 public:
  explicit SignalProcessor(SignalProcessingConfig cfg) : cfg_(cfg) {}

  std::vector<RadarDetection> detect_radar(const std::vector<RadarHit>& hits) const;
  std::vector<AcousticDetection> detect_acoustic(const std::vector<AcousticHit>& hits) const;

 private:
  SignalProcessingConfig cfg_{};
};

}  // namespace mst

