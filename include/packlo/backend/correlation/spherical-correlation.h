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

		void getStatistics(common::StatisticsManager* manager) const noexcept;

	private:
		void convertSignalValues(double *signal_values, 
				const int bw);
		void convertSignalCoeff(double *signal_coeff, 
				const int bw);
	  const std::string kReferenceName = "SPH-Correlation";
	  const std::string kSignalKey = "signal_values";
	  const std::string kCoeffKey = "signal_coeff";
	  common::StatisticsManager statistics_manager_;                              
};

} // namespace backend
