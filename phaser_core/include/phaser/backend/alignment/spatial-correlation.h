#ifndef PACKLO_BACKEND_ALIGNMENT_SPATIAL_CORRELATION_H_
#define PACKLO_BACKEND_ALIGNMENT_SPATIAL_CORRELATION_H_

#include "phaser/backend/alignment/base-spatial-correlation.h"

namespace alignment {

class SpatialCorrelation : public BaseSpatialCorrelation {
public:
  void correlateSignals(double* const f, double* const g) override;
};

}  // namespace alignment

#endif  // PACKLO_BACKEND_ALIGNMENT_SPATIAL_CORRELATION_H_
