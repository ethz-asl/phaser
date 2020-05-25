#ifndef PHASER_BACKEND_CORRELATION_SPHERICAL_CORRELATION_H_
#define PHASER_BACKEND_CORRELATION_SPHERICAL_CORRELATION_H_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <fftw3/fftw3.h>

#include "phaser/common/statistics-manager.h"
#include "phaser/model/function-value.h"

namespace phaser_core {

using SampledSignal = std::vector<double>;

class SphericalCorrelation {
 public:
  explicit SphericalCorrelation(
      const uint32_t bw = 100, const uint32_t zero_padding = 0);
  virtual ~SphericalCorrelation();
  void correlateSignals(
      const std::vector<model::FunctionValue>& f1,
      const std::vector<model::FunctionValue>& f2);

  virtual void correlateSampledSignals(
      const std::vector<SampledSignal>& f1,
      const std::vector<SampledSignal>& f2);

  void getStatistics(common::StatisticsManager* manager) const noexcept;
  std::vector<double> getCorrelation() const noexcept;
  uint32_t getBandwidth() const noexcept;

 protected:
  void convertSignalValues(double* signal_values, const int bw);
  void convertSignalCoeff(double* signal_coeff, const int bw);
  void retrieveInterpolation(
      const std::vector<model::FunctionValue>& f,
      std::vector<double>* interpolation);
  void initializeAll(const uint32_t bw);
  void performSphericalTransforms(
      const std::vector<double>& f1, const std::vector<double>& f2);
  void correlate();
  void inverseTransform();

  const std::string kReferenceName = "SPH-Correlation";
  const std::string kSignalKey = "signal_values";
  const std::string kCoeffKey = "signal_coeff";
  const double two_pi_ = 2 * M_PI;
  const uint32_t zero_padding_;
  const uint32_t bw_out_;
  uint32_t bw_;
  uint32_t so3_bw_;
  std::vector<double> corr_;
  common::StatisticsManager statistics_manager_;

  fftw_plan dct_plan_;
  fftw_plan fft_plan_;
  fftw_plan inverse_so3_;
  fftw_complex* workspace1_;
  fftw_complex* workspace2_;
  double* workspace3_;

  fftw_complex* workspace2_out_;
  double* workspace3_out_;

  fftw_complex* so3_sig_;
  double* seminaive_naive_tablespace_;
  double** seminaive_naive_table_;

  double* weights_;
  int rank_, howmany_, howmany_rank_, istride_, idist_, ostride_, odist_;
  fftw_iodim dims_[1], howmany_dims_[1];
  int na_[2], inembed_[2], onembed_[2];

  double* tmp_coef_[2];
  double* sig_coef_[2];
  double* pat_coef_[2];
  fftw_complex* so3_coef_;
  std::vector<double> so3_mag_sig_;
};

using SphericalCorrelationPtr = std::unique_ptr<SphericalCorrelation>;

}  // namespace phaser_core

#endif  // PHASER_BACKEND_CORRELATION_SPHERICAL_CORRELATION_H_
