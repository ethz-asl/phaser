#include "phaser/distribution/gaussian-mixture.h"

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

TEST_F(GaussianMixtureTest, CalcGMComponentsFromGaussiansTest) {
  Eigen::VectorXd mu1(3);
  Eigen::VectorXd mu2(3);
  Eigen::MatrixXd cov1(3, 3);
  Eigen::MatrixXd cov2(3, 3);
  mu1 << 1, 2, 3;
  cov1 << 1, 0, 0, 0, 2, 0, 0, 0, 3;
  mu2 << 2, 2, 1;
  cov2 << 4, 0, 0, 0, 5, 0, 0, 0, 6;

  std::vector<common::Gaussian> gaussians;
  gaussians.emplace_back(common::Gaussian(mu1, cov1));
  gaussians.emplace_back(common::Gaussian(mu2, cov2));

  Eigen::VectorXd gm_weights(2);
  gm_weights << 0.3, 0.7;
  common::GaussianMixture gm(gaussians, gm_weights);

  Eigen::Vector3d true_mean(1.7, 2.0, 1.6);
  Eigen::MatrixXd true_cov(3, 3);
  true_cov << 3.31, 0, -0.42, 0, 4.1, 0, -0.42, 0, 5.94;

  EXPECT_NEAR_EIGEN(gm.getMixtureMean(), true_mean, 1e-4);
  EXPECT_NEAR_EIGEN(gm.getMixtureCov(), true_cov, 1e-4);
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
