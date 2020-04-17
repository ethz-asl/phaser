#include "phaser/backend/alignment/base-spatial-correlation.h"

namespace alignment {

double* BaseSpatialCorrelation::correlateSignals(
    Eigen::VectorXd* const f, Eigen::VectorXd* const g) {
  correlateSignals(f->data(), g->data());
}

}  // namespace alignment
