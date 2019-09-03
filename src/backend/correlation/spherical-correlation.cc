#include <packlo/backend/correlation/spherical-correlation.h>

extern "C" {
#include <soft/wrap_fftw.h>
}

#include <glog/logging.h>

namespace backend {

std::array<double, 3> SphericalCorrelation::correlateSignals(
    const std::vector<float>& f1,
    const std::vector<float>& f2, const int bw) {
  double alpha, beta, gamma = 0;
  const int is_real = 1;
   
  std::size_t n_signal = f1.size();
  std::size_t n_pattern = f2.size();
  double* signal = new double[n_signal];
  double* pattern = new double[n_pattern];

  for (std::size_t i = 0; i < n_signal; ++i) {
    signal[i] = f1[i];
  }

  for (std::size_t i = 0; i < n_pattern; ++i) {
    pattern[i] = f2[i];
  }

  VLOG(1) << "starting correlation...";
  softFFTWCor2(bw, signal, pattern, 
      &alpha, &beta, &gamma, is_real);

  VLOG(1) << "done: " << alpha << ", " << beta << ", " << gamma;
  std::array<double, 3> zyz {alpha, beta, gamma};
  return zyz;
}

}
