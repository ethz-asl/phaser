#include "phaser/common/test-worker.h"

#include <chrono>
#include <glog/logging.h>
#include <thread>

namespace common {

TestWorker::TestWorker(const uint8_t id) : id_(id) {}

void TestWorker::run() {
  VLOG(1) << "Starting test worker " << id_ << ".";
  std::this_thread::sleep_for(std::chrono::seconds(5));
  VLOG(1) << "Finished test worker " << id_ << ".";
  is_completed_ = true;
}

}  // namespace common
