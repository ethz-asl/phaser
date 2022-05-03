#include "phaser/distribution/bingham-opt-mle.h"

#include <glog/logging.h>
#include <nlopt.hpp>
#include <vector>

#include "phaser/distribution/bingham-objective.h"

namespace common {

static double objective_wrapper(
    const std::vector<double>& x, std::vector<double>& grad, void* obj) {
  BinghamObjective* objective = reinterpret_cast<BinghamObjective*>(obj);
  // if (!grad.empty()) objective->calculateGrad(grad);
  return objective->optimize(x);
}

Eigen::VectorXd BinghamOptMLE::compute(const Eigen::VectorXd& omega) {
  BinghamObjective objective(omega);

  nlopt::opt opt(nlopt::LN_COBYLA, 3);
  opt.set_min_objective(objective_wrapper, &objective);
  opt.set_xtol_rel(1e-4);

  std::vector<double> t_init = {-1.0, -1.0, -1.0};

  double solminf;
  VLOG(1) << "starting to optimize";
  try {
    nlopt::result result = opt.optimize(t_init, solminf);
    VLOG(3) << "Optimization succeeded with " << static_cast<int>(result);
  } catch (std::exception& e) {
    LOG(FATAL) << "Optimization Failed!";
  }
  return Eigen::Vector4d(t_init[0], t_init[1], t_init[2], 0);
}

}  // namespace common
