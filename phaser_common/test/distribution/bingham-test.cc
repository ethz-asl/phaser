#include "packlo/distribution/bingham.h"
#include "packlo/common/test/testing-entrypoint.h"
#include "packlo/common/test/testing-predicates.h"

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <memory>
#include <random>

namespace common {

class BinghamTest : public ::testing::Test {
 public:
  BinghamTest() = default;

 protected:
  virtual void SetUp() {}
};

TEST_F(BinghamTest, constructorTest) {
  // EXPECT_NEAR_EIGEN(gauss.getMean(), true_mean, 1e-4);
  // EXPECT_NEAR_EIGEN(gauss.getCov(), true_cov, 1e-4);
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
