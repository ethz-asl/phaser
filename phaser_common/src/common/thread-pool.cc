#include "phaser/common/thread-pool.h"

namespace common {

void ThreadPool::run_and_wait_all() {
  threads_.clear();
  // Create worker threads.
  for (BaseWorkerPtr worker : workers_) {
    threads_.emplace_back(new std::thread(std::bind(&BaseWorker::run, worker)));
  }

  // Join all threads.
  for (const std::unique_ptr<std::thread>& th : threads_) {
    th->join();
  }
  workers_.clear();
}

void ThreadPool::add_worker(BaseWorkerPtr worker) {
  workers_.emplace_back(worker);
}

}  // namespace common
