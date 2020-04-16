#include "phaser/backend/alignment/base-spatial-correlation.h"

namespace alignment {

double* BaseSpatialCorrelation::correlateSignals(
    Eigen::VectorXd* f, Eigen::VectorXd* g) {
  correlateSignals(f->data(), g->data());
}

}  // namespace alignment
