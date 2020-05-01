#include "phaser/common/base-worker.h"

namespace common {

bool BaseWorker::isCompleted() const {
  return is_completed_;
}

}  // namespace common
