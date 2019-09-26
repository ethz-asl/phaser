#include "packlo/backend/correlation/spherical-correlation.h"

extern "C" {
#include <soft/wrap_fftw.h>
}

#include <glog/logging.h>

namespace backend {

SphericalCorrelation::SphericalCorrelation() 
		: statistics_manager_(kReferenceName) {
}

void SphericalCorrelation::correlateSignals(
    const std::vector<float>& f1,
    const std::vector<float>& f2, const int bw, 
		std::array<double, 3>* const zyz) {
  double alpha, beta, gamma, maxcoeff = 0.0;
  constexpr int is_real = 1;
   
  const std::size_t n_signal = f1.size();
  const std::size_t n_pattern = f2.size();
  double* signal = new double[n_signal];
  double* pattern = new double[n_pattern];

	// TODO(lbern): change f1 and f2 to be double? 
  for (std::size_t i = 0; i < n_signal; ++i) {
    signal[i] = f1[i];
  }

  for (std::size_t i = 0; i < n_pattern; ++i) {
    pattern[i] = f2[i];
  }

  VLOG(2) << "starting correlation with " << n_signal << " signal coeff and "
		<< n_pattern << " pattern coeff.";
	double *signal_values;
  softFFTWCor2(bw, signal, pattern, 
      &alpha, &beta, &gamma, &maxcoeff, &signal_values, is_real);
  VLOG(2) << "done, result: " << alpha << ", " << beta << ", " << gamma;

	(*zyz)[0] = alpha;
	(*zyz)[1] = beta;
	(*zyz)[2] = gamma;

	delete [] signal;
	delete [] pattern;

	CHECK_NOTNULL(signal_values);
	convertSignalValues(signal_values, bw);
	delete [] signal_values;
}

void SphericalCorrelation::getStatistics(
		common::StatisticsManager* manager)
	 const noexcept {
 manager->mergeManager(statistics_manager_);
}

void SphericalCorrelation::convertSignalValues(
		double *signal_values, const int bw) {
	VLOG(1) << "ADDING CORR KEYS FOR BW " << bw;
	const std::size_t n_values = 8*bw*bw*bw;
	for (std::size_t i = 0u; i < n_values; ++i) {
		statistics_manager_.emplaceValue(kSignalKey, signal_values[i]);   
	}
}

}
