#ifndef PHASER_BACKEND_CORRELATION_SPHERICAL_CORRELATION_LAPLACE_H_
#define PHASER_BACKEND_CORRELATION_SPHERICAL_CORRELATION_LAPLACE_H_

#include <utility>
#include <vector>

#include "phaser/backend/correlation/spherical-correlation.h"
#include "phaser/backend/fusion/laplace-pyramid.h"

namespace correlation {

class SphericalCorrelationLaplace : public SphericalCorrelation {
 public:
  explicit SphericalCorrelationLaplace(const uint32_t bw = 100);
  virtual ~SphericalCorrelationLaplace() = default;
  void correlateSampledSignals(
      const std::vector<SampledSignal>& f,
      const std::vector<SampledSignal>& g) override;

 protected:
  std::pair<fftw_complex*, fftw_complex*> performFFTandShift(
      const SampledSignal& f1, const SampledSignal& f2,
      const uint32_t n_coeffs);
  fftw_complex* convertCoeffArrays(
      double* f_real, double* f_imag, const uint32_t n_coeffs);
  void setFusedCoefficients(
      const std::vector<fusion::complex_t>& signal,
      const std::vector<fusion::complex_t>& pattern, const uint32_t n_coeffs);
  void setFusedCoefficients(
      fftw_complex* signal, fftw_complex* pattern, const uint32_t n_coeffs);

  void shiftSignals(const uint32_t n_points);
  void inverseShiftSignals(const uint32_t n_points);

  fusion::LaplacePyramid laplace_;
};

}  // namespace correlation

#endif  // PHASER_BACKEND_CORRELATION_SPHERICAL_CORRELATION_LAPLACE_H_
