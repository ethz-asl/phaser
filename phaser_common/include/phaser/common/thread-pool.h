#ifndef PHASER_COMMON_THREAD_POOL_H_
#define PHASER_COMMON_THREAD_POOL_H_

#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "phaser/common/base-worker.h"

namespace common {

class ThreadPool {
 public:
  void run_and_wait_all();
  void add_worker(BaseWorkerPtr worker);

 private:
  std::vector<std::unique_ptr<std::thread>> threads_;
  std::vector<BaseWorkerPtr> workers_;
};

}  // namespace common

#endif  // PHASER_COMMON_THREAD_POOL_H_
