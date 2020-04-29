#include "phaser/backend/correlation/base-spatial-correlation.h"

namespace correlation {

double* BaseSpatialCorrelation::correlateSignals(
    Eigen::VectorXd* const f, Eigen::VectorXd* const g) {
  correlateSignals(f->data(), g->data());
}

}  // namespace correlation
