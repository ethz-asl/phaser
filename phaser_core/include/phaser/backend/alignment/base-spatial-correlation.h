#ifndef PACKLO_BACKEND_ALIGNMENT_BASE_SPATIAL_CORRELATION_H_
#define PACKLO_BACKEND_ALIGNMENT_BASE_SPATIAL_CORRELATION_H_

#include <memory>

#include <Eigen/Dense>

namespace alignment {

class BaseSpatialCorrelation {
public:
  virtual double* correlateSignals(double* f, double* g) = 0;
};

using BaseSpatialCorrelationPtr = std::unique_ptr<BaseSpatialCorrelation>;

} // namespace alignment

#endif  // PACKLO_BACKEND_ALIGNMENT_BASE_SPATIAL_CORRELATION_H_
