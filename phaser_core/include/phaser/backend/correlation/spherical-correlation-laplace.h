#ifndef PHASER_BACKEND_CORRELATION_SPHERICAL_CORRELATION_LAPLACE_H_
#define PHASER_BACKEND_CORRELATION_SPHERICAL_CORRELATION_LAPLACE_H_

#include <utility>
#include <vector>

#include "phaser/backend/correlation/spherical-correlation.h"
#include "phaser/backend/fusion/laplace-pyramid.h"

namespace phaser_core {

class SphericalCorrelationLaplace : public SphericalCorrelation {
 public:
  explicit SphericalCorrelationLaplace(
      const uint32_t bw = 100, const uint32_t zero_padding = 0);
  virtual ~SphericalCorrelationLaplace() = default;
  void correlateSampledSignals(
      const std::vector<SampledSignal>& f,
      const std::vector<SampledSignal>& g) override;

 protected:
  void extractTransformedChannels(
      const std::vector<SampledSignal>& fs,
      const std::vector<SampledSignal>& gs,
      std::vector<fftw_complex*>* f_channels,
      std::vector<fftw_complex*>* g_channels);
  std::pair<fftw_complex*, fftw_complex*> performFFTandShift(
      const SampledSignal& f1, const SampledSignal& f2,
      const uint32_t n_coeffs);
  fftw_complex* convertCoeffArrays(
      double* f_real, double* f_imag, const uint32_t n_coeffs);
  void setFusedCoefficients(
      const std::vector<complex_t>& signal,
      const std::vector<complex_t>& pattern, const uint32_t n_coeffs);
  void setFusedCoefficients(
      fftw_complex* signal, fftw_complex* pattern, const uint32_t n_coeffs);
  void freeChannels(std::vector<fftw_complex*>* channels);

  void shiftSignals(const uint32_t n_points);
  void inverseShiftSignals(const uint32_t n_points);

  LaplacePyramid laplace_;
};

}  // namespace phaser_core

#endif  // PHASER_BACKEND_CORRELATION_SPHERICAL_CORRELATION_LAPLACE_H_
