#ifndef PHASER_COMMON_THREAD_POOL_H_
#define PHASER_COMMON_THREAD_POOL_H_

#include <functional>
#include <memory>
#include <thread>
#include <vector>

namespace common {

class ThreadPool {
 public:
  explicit ThreadPool(const std::size_t num_threads);

  void run_all();
  void add_task(std::function<void()> func);

 private:
  std::size_t num_threads_;
  std::vector<std::unique_ptr<std::thread>> threads_;
};

}  // namespace common

#endif  // PHASER_COMMON_THREAD_POOL_H_
