#include "packlo/distribution/gaussian-mixture.h"
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

class GaussianMixtureTest : public ::testing::Test {
 public:
  GaussianMixtureTest() = default;

 protected:
  virtual void SetUp() {}
};

TEST_F(GaussianMixtureTest, CalcGMComponentsTest) {
  Eigen::MatrixXd means(2, 2);
  means << 1, 2, 3, 4;
  std::vector<Eigen::MatrixXd> covs;
  covs.emplace_back(Eigen::MatrixXd::Identity(2, 2));
  covs.emplace_back(Eigen::MatrixXd::Identity(2, 2));
  Eigen::VectorXd weights(2);
  weights << 0.4, 0.6;

  GaussianMixture gm(means, covs, weights);
  Eigen::Vector2d true_mean(1.6, 3.6);
  Eigen::MatrixXd true_cov(2, 2);
  true_cov << 1.24, 0.24, 0.24, 1.24;

  EXPECT_NEAR_EIGEN(gm.getMixtureMean(), true_mean, 1e-4);
  EXPECT_NEAR_EIGEN(gm.getMixtureCov(), true_cov, 1e-4);
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
