#pragma once

#include <vector>
#include <array>

namespace backend {

class SphericalCorrelation {
public:
  std::array<double, 3> correlateSignals(const std::vector<float>& f1,
      const std::vector<float>& f2, const int bw);
};

}
