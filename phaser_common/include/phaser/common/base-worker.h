#ifndef PHASER_COMMON_BASE_WORKER_H_
#define PHASER_COMMON_BASE_WORKER_H_

#include <memory>

namespace common {

class BaseWorker {
 public:
  virtual void run() = 0;
  bool isCompleted() const;

 protected:
  bool is_completed_ = false;
};

using BaseWorkerPtr = std::shared_ptr<BaseWorker>;

}  // namespace common

#endif  // PHASER_COMMON_BASE_WORKER_H_
