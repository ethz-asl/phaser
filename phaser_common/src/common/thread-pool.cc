#include "phaser/common/thread-pool.h"

namespace common {

ThreadPool::ThreadPool(const std::size_t num_threads)
    : num_threads_(num_threads) {}

void ThreadPool::run_all() {}

void ThreadPool::add_task(std::function<void()> func) {}

}  // namespace common
