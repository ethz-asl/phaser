#include "phaser/distribution/bingham-objective.h"

#include <glog/logging.h>

#include "phaser/distribution/bingham.h"

namespace common {

BinghamObjective::BinghamObjective(const Eigen::Vector4d& omega)
    : omega_(omega) {}

double BinghamObjective::optimize(const std::vector<double>& x) {
  CHECK_EQ(x.size(), 3);

  Eigen::Vector4d Z(x[0], x[1], x[2], 0.0);
  const double a = Bingham::computeF(Z);
  const Eigen::Vector4d b = Bingham::computeDF(Z);
  Eigen::Vector4d error = (b.array() / a) - omega_.array();
  return error.lpNorm<2>();
}

}  // namespace common
