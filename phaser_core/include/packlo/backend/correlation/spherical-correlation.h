#ifndef PACKLO_BACKEND_CORRELATION_SPHERICAL_CORRELATION_H_
#define PACKLO_BACKEND_CORRELATION_SPHERICAL_CORRELATION_H_

#include "packlo/common/statistics-manager.h"
#include "packlo/model/function-value.h"

#include <array>
#include <string>
#include <vector>

namespace backend {

class SphericalCorrelation {
 public:
  SphericalCorrelation();
  void correlateSignals(
      const std::vector<model::FunctionValue>& f1,
      const std::vector<model::FunctionValue>& f2, const int bw,
      std::array<double, 3>* const zyz);

  void getStatistics(common::StatisticsManager* manager) const noexcept;
  std::vector<double> getCorrelation() const noexcept;
  std::array<double, 3> getZYZFromIndex(const uint32_t index) const noexcept;

 private:
  void convertSignalValues(double* signal_values, const int bw);
  void convertSignalCoeff(double* signal_coeff, const int bw);
  void retrieveInterpolation(
      const std::vector<model::FunctionValue>& f,
      std::vector<double>* interpolation);
  void convertSO3toZYZ(
      const uint32_t loc, const uint32_t bw,
      std::array<double, 3>* const zyz) const;

  const std::string kReferenceName = "SPH-Correlation";
  const std::string kSignalKey = "signal_values";
  const std::string kCoeffKey = "signal_coeff";
  const double two_pi_ = 2 * M_PI;
  uint32_t bw_;
  std::vector<double> corr_;
  common::StatisticsManager statistics_manager_;
};

}  // namespace backend

#endif  // PACKLO_BACKEND_CORRELATION_SPHERICAL_CORRELATION_H_
