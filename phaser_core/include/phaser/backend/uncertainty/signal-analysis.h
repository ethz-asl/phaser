#ifndef PHASER_BACKEND_UNCERTAINTY_SIGNAL_ANALYSIS_H_
#define PHASER_BACKEND_UNCERTAINTY_SIGNAL_ANALYSIS_H_

#include <cstdint>
#include <vector>

namespace phaser_core {

class SignalAnalysis {
 public:
  static double stdDev(
      const std::vector<double>& vec, const double mean, const uint32_t from,
      const uint32_t to);
};

}  // namespace phaser_core

#endif  // PHASER_BACKEND_UNCERTAINTY_SIGNAL_ANALYSIS_H_
