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
  Eigen::MatrixXd M = Eigen::MatrixXd::Identity(2, 2);
  Eigen::Vector2d Z(-3, 0);
  common::Bingham bingham(Z, M);

  Eigen::Vector2d true_mode(0, 1);
  Eigen::MatrixXd true_moment(2, 2);
  true_moment << 0.201933380584355, 0, 0, 0.798066619415645;

  EXPECT_NEAR_EIGEN(bingham.mode(), true_mode, 1e-4);
  EXPECT_NEAR_EIGEN(bingham.moment(), true_moment, 1e-4);
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
