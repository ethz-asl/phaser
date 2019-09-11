#include "packlo/backend/correlation/spherical-correlation.h"

extern "C" {
#include <soft/wrap_fftw.h>
}

#include <glog/logging.h>

namespace backend {

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
  softFFTWCor2(bw, signal, pattern, 
      &alpha, &beta, &gamma, &maxcoeff, is_real);
  VLOG(2) << "done, result: " << alpha << ", " << beta << ", " << gamma;

	(*zyz)[0] = alpha;
	(*zyz)[1] = beta;
	(*zyz)[2] = gamma;

	delete [] signal;
	delete [] pattern;
}

}
