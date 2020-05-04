#ifndef PHASER_BACKEND_CORRELATION_SPHERICAL_CORRELATION_H_
#define PHASER_BACKEND_CORRELATION_SPHERICAL_CORRELATION_H_

#include "phaser/common/statistics-manager.h"
#include "phaser/model/function-value.h"

#include <array>
#include <string>
#include <vector>

namespace correlation {

class SphericalCorrelation {
 public:
  SphericalCorrelation();
  void correlateSignals(
      const std::vector<model::FunctionValue>& f1,
      const std::vector<model::FunctionValue>& f2, const int bw);

  void correlateSampledSignals(
      const int bw, std::vector<double>& f1, std::vector<double>& f2);

  void getStatistics(common::StatisticsManager* manager) const noexcept;
  std::vector<double> getCorrelation() const noexcept;
  uint32_t getBandwidth() const noexcept;

 private:
  void convertSignalValues(double* signal_values, const int bw);
  void convertSignalCoeff(double* signal_coeff, const int bw);
  void retrieveInterpolation(
      const std::vector<model::FunctionValue>& f,
      std::vector<double>* interpolation);

  const std::string kReferenceName = "SPH-Correlation";
  const std::string kSignalKey = "signal_values";
  const std::string kCoeffKey = "signal_coeff";
  const double two_pi_ = 2 * M_PI;
  uint32_t bw_;
  std::vector<double> corr_;
  common::StatisticsManager statistics_manager_;
};

}  // namespace correlation

#endif  // PHASER_BACKEND_CORRELATION_SPHERICAL_CORRELATION_H_
