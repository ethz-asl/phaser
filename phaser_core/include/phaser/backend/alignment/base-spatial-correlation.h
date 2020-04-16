#ifndef PACKLO_BACKEND_ALIGNMENT_BASE_SPATIAL_CORRELATION_H_
#define PACKLO_BACKEND_ALIGNMENT_BASE_SPATIAL_CORRELATION_H_

#include <Eigen/Dense>

namespace alignment {

class BaseSpatialCorrelation {
public:
  virtual double* correlateSignals(double* f, double* g) = 0;
};

} // namespace alignment

#endif  // PACKLO_BACKEND_ALIGNMENT_BASE_SPATIAL_CORRELATION_H_
