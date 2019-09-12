#pragma once

#include <vector>
#include <array>

namespace backend {

class SphericalCorrelation {
	public:
    void correlateSignals(const std::vector<float>& f1,
      const std::vector<float>& f2, const int bw, 
			std::array<double, 3>* const zyz);
};

} // namespace backend
