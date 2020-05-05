#ifndef PHASER_COMMON_BASE_WORKER_H_
#define PHASER_COMMON_BASE_WORKER_H_

#include <functional>
#include <memory>
#include <vector>

#include "phaser/model/function-value.h"

namespace common {

class BaseWorker {
 public:
  virtual void run() = 0;
  bool isCompleted() const;

  void convertFunctionValues(
      const std::vector<model::FunctionValue>& f,
      const std::function<double(const model::FunctionValue&)>& func,
      std::vector<double>* interpolation);

 protected:
  bool is_completed_ = false;
};

using BaseWorkerPtr = std::shared_ptr<BaseWorker>;

}  // namespace common

#endif  // PHASER_COMMON_BASE_WORKER_H_
