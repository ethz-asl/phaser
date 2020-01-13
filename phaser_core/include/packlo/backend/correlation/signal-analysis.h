#ifndef PACKLO_BACKEND_CORRELATION_SIGNAL_ANALYSIS_H_
#define PACKLO_BACKEND_CORRELATION_SIGNAL_ANALYSIS_H_

#include <cstdint>
#include <vector>

namespace correlation {

class SignalAnalysis {
 public:
  static double stdDev(
      const std::vector<double>& vec, const double mean, const uint32_t from,
      const uint32_t to);
};

}  // namespace correlation

#endif  // PACKLO_BACKEND_CORRELATION_SIGNAL_ANALYSIS_H_
