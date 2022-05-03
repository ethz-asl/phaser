#include "phaser/distribution/gaussian.h"

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <memory>
#include <random>

#include "phaser/common/test/testing-entrypoint.h"
#include "phaser/common/test/testing-predicates.h"

namespace common {

class GaussianTest : public ::testing::Test {
 public:
  GaussianTest() = default;

 protected:
  virtual void SetUp() {}
};

TEST_F(GaussianTest, calcMeanAndCovTest) {
  Eigen::MatrixXd samples(3, 3);
  Eigen::VectorXd weights(3);
  samples << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  weights << 0.2, 0.6, 0.2;
  common::Gaussian gauss(samples, weights);

  Eigen::VectorXd true_mean(3);
  Eigen::MatrixXd true_cov(3, 3);
  true_mean << 2, 5, 8;
  true_cov << 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4;

  EXPECT_NEAR_EIGEN(gauss.getMean(), true_mean, 1e-4);
  EXPECT_NEAR_EIGEN(gauss.getCov(), true_cov, 1e-4);
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
