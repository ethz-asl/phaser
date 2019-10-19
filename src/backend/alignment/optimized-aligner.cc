#include "packlo/backend/alignment/optimized-aligner.h"
#include "packlo/backend/alignment/function-objective.h"
#include "packlo/backend/alignment/cloud-objective.h"

#include <glog/logging.h>
#include <nlopt.hpp>
#include <cmath>

DEFINE_double(nlopt_xtol, 1e-4, 
    "Defines the tolerance for the optimization.");

namespace alignment {

static double objective_wrapper(const std::vector<double>& x, 
    std::vector<double>& grad, void *obj)
{
    BaseObjective* objective = reinterpret_cast<BaseObjective*>(obj);
    if (!grad.empty()) objective->calculateGrad(grad);
    return objective->optimize(x);
}

OptimizedAligner::OptimizedAligner() {
  //objective_ = std::make_unique<FunctionObjective>();
  objective_ = std::make_unique<CloudObjective>();
}

void OptimizedAligner::alignRegistered(
    const model::PointCloud& cloud_prev, 
    const std::vector<model::FunctionValue>& f_prev, 
    const model::PointCloud& cloud_reg,
    const std::vector<model::FunctionValue>& f_reg, 
    common::Vector_t* xyz) {
  CHECK(xyz);

  objective_->setPrevious(cloud_prev, f_prev);
  objective_->setCurrent(cloud_reg, f_reg);

  nlopt::opt opt(nlopt::LN_COBYLA, 3);
  opt.set_min_objective(objective_wrapper, &(*objective_));
  opt.set_xtol_rel(FLAGS_nlopt_xtol);

  std::vector<double> t_init = {0.0, 0.0, 0.0};
  double solminf;
  VLOG(1) << "starting to optimize";
  try {
    nlopt::result result = opt.optimize(t_init, solminf);
    VLOG(1) << "Optimization succeeded with " 
      << static_cast<int>(result);
  } catch (std::exception& e) {
    VLOG(1) << "Optimization Failed!";
  }
  
  (*xyz)(0) = t_init[0];
  (*xyz)(1) = t_init[1];
  (*xyz)(2) = t_init[2];
}

std::vector<double> OptimizedAligner::getCorrelation() const {
  return std::vector<double>();
}

} // namespace alignment
