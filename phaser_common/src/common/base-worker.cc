#include "phaser/common/base-worker.h"

namespace common {

bool BaseWorker::isCompleted() const {
  return is_completed_;
}
void BaseWorker::convertFunctionValues(
    const std::vector<model::FunctionValue>& f,
    const std::function<double(const model::FunctionValue&)>& func,
    std::vector<double>* interpolation) {
  std::transform(
      f.cbegin(), f.cend(), std::back_inserter(*interpolation), func);
}

}  // namespace common
