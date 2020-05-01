#include <chrono>
#include <cmath>
#include <memory>
#include <random>

#include <Eigen/Dense>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "phaser/common/base-worker.h"
#include "phaser/common/test-worker.h"
#include "phaser/common/test/testing-entrypoint.h"
#include "phaser/common/test/testing-predicates.h"
#include "phaser/common/thread-pool.h"

namespace common {

class ThreadPoolTest : public ::testing::Test {};

TEST_F(ThreadPoolTest, TestWorkerTest) {
  BaseWorkerPtr worker_1 = std::make_shared<TestWorker>(1);
  BaseWorkerPtr worker_2 = std::make_shared<TestWorker>(2);
  EXPECT_NE(worker_1, nullptr);
  EXPECT_NE(worker_2, nullptr);
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
