#pragma once

#include "packlo/common/statistics-manager.h"


#include <vector>
#include <array>

namespace backend {

class SphericalCorrelation {
	public:
		SphericalCorrelation();
    void correlateSignals(const std::vector<float>& f1,
      const std::vector<float>& f2, const int bw, 
			std::array<double, 3>* const zyz);

		const common::StatisticsManager& getStatistics() const noexcept;

	private:
		void convertSignalValues(double *signal_values, 
				const int bw);
	  const std::string kReferenceName = "SPH-Correlation";
	  const std::string kSignalKey = "signal_values";
	  common::StatisticsManager statistics_manager_;                              
};

} // namespace backend
