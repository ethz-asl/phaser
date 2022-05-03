#include "phaser/common/thread-pool.h"

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <memory>
#include <random>

#include "phaser/common/test-worker.h"
#include "phaser/common/test/testing-entrypoint.h"
#include "phaser/common/test/testing-predicates.h"

namespace common {

class ThreadPoolTest : public ::testing::Test {};

TEST_F(ThreadPoolTest, TestWorkerTest) {
  BaseWorkerPtr worker_1 = std::make_shared<TestWorker>(1);
  BaseWorkerPtr worker_2 = std::make_shared<TestWorker>(2);
  EXPECT_NE(worker_1, nullptr);
  EXPECT_NE(worker_2, nullptr);
  EXPECT_TRUE(!worker_1->isCompleted());
  EXPECT_TRUE(!worker_2->isCompleted());

  ThreadPool tp;
  tp.add_worker(worker_1);
  tp.add_worker(worker_2);

  tp.run_and_wait_all();

  EXPECT_TRUE(worker_1->isCompleted());
  EXPECT_TRUE(worker_2->isCompleted());
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
