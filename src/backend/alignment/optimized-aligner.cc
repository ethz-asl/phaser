#include "packlo/backend/alignment/optimized-aligner.h"

#include <glog/logging.h>
#include <nlopt.hpp>
#include <cmath>

namespace alignment {

static double myfunc(const std::vector<double>& x, 
    std::vector<double>& grad, void *obj)
{
    Objective* objective = reinterpret_cast<Objective*>(obj);
    if (!grad.empty()) objective->calculateGrad(grad);
    return objective->optimize(x, grad);
}

common::Vector_t OptimizedAligner::alignRegistered(
    const model::PointCloud& cloud_prev, 
    const std::vector<model::FunctionValue>& f_prev, 
    const model::PointCloud& cloud_reg,
    const std::vector<model::FunctionValue>& f_reg) {

  objective_.setPrevious(cloud_prev, f_prev);
  objective_.setCurrent(cloud_reg, f_reg);

  nlopt::opt opt(nlopt::LN_COBYLA, 3);
  //std::vector<double> lb(2);
  //lb[0] = -HUGE_VAL; lb[1] = 0;
  //opt.set_lower_bounds(lb);
  opt.set_min_objective(myfunc, &objective_);
  opt.set_xtol_rel(1e-4);

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
  
  return common::Vector_t(t_init[0], t_init[1], t_init[2]);
}

} // namespace alignment
