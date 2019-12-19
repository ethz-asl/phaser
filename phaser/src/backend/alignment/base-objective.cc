#include "packlo/backend/alignment/base-objective.h"

#include <Eigen/Dense>
#include <glog/logging.h>

namespace alignment {

void BaseObjective::setPrevious(const model::PointCloud& cloud_prev, 
    const std::vector<model::FunctionValue>& f_prev) {
  cloud_prev_ = &cloud_prev;
  f_prev_ = &f_prev;
}

void BaseObjective::setCurrent(const model::PointCloud& cloud_cur, 
    const std::vector<model::FunctionValue>& f_cur) {
  cloud_cur_ = &cloud_cur;
  f_cur_ = &f_cur;
}

} // namespace alignment
