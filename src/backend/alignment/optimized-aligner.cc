#include "packlo/backend/alignment/optimized-aligner.h"

#include <cmath>
#include <nlopt.hpp>

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

  nlopt::opt opt(nlopt::LD_MMA, 3);
  std::vector<double> lb(2);
  lb[0] = -HUGE_VAL; lb[1] = 0;
  opt.set_lower_bounds(lb);
  opt.set_min_objective(myfunc, &objective_);

  return common::Vector_t();
}

} // namespace alignment
