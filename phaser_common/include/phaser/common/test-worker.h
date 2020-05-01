#ifndef PHASER_COMMON_TEST_WORKER_H_
#define PHASER_COMMON_TEST_WORKER_H_

#include <cstdint>

#include "phaser/common/base-worker.h"

namespace common {

class TestWorker : public BaseWorker {
 public:
  explicit TestWorker(const uint8_t id);
  void run() override;

 private:
  uint8_t id_;
};

}  // namespace common

#endif  // PHASER_COMMON_TEST_WORKER_H_
