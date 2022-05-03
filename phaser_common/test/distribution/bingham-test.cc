#include "phaser/distribution/bingham.h"

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <memory>
#include <random>

#include "phaser/common/rotation-utils.h"
#include "phaser/common/test/testing-entrypoint.h"
#include "phaser/common/test/testing-predicates.h"

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

TEST_F(BinghamTest, fitTest) {
  Eigen::MatrixXd samples(4, 3);
  samples << 0.956937406927354, 0.960768653534504, 0.957187982859276,
      0.058856783978165, 0.065511336552757, 0.050483920758591,
      0.168490940966118, 0.177531610326744, 0.190308650775464,
      0.228948642746032, 0.202766827916600, 0.212191320874942;

  common::Bingham bingham = common::Bingham::fit(samples);
  Eigen::VectorXd true_mode(4);
  true_mode << 0.958412197672200, 0.058290946520010, 0.178798556011915,
      0.214660898337351;
  Eigen::MatrixXd true_moment(4, 4);
  true_moment << 0.910013003508880, 0.055153324084561, 0.169173805803000,
      0.203105375179439, 0.055153324084561, 0.006547337120034,
      0.010288469514638, 0.012352543762725, 0.169173805803000,
      0.010288469514638, 0.034754366311497, 0.037889670084817,
      0.203105375179439, 0.012352543762725, 0.037889670084817,
      0.048685293059589;
  EXPECT_NEAR_EIGEN(bingham.mode(), true_mode, 1e-4);
  EXPECT_NEAR_EIGEN(bingham.moment(), true_moment, 1e-4);
}

TEST_F(BinghamTest, fitWeightedTest) {
  Eigen::MatrixXd samples(4, 3);
  samples << 0.956937406927354, 0.960768653534504, 0.957187982859276,
      0.058856783978165, 0.065511336552757, 0.050483920758591,
      0.168490940966118, 0.177531610326744, 0.190308650775464,
      0.228948642746032, 0.202766827916600, 0.212191320874942;
  Eigen::RowVector3d weights(0.2, 0.7, 0.1);

  common::Bingham bingham = common::Bingham::fit(samples, weights);
  Eigen::VectorXd true_mode(4);
  true_mode << 0.959724413089442, 0.062683254650737, 0.177016105317784,
      0.208962099349714;
  EXPECT_NEAR_EIGEN(bingham.mode(), true_mode, 1e-4);
}

TEST_F(BinghamTest, fitWeightedQuaternionTest) {
  // w x y z
  Eigen::Quaterniond q1(
      0.956937406927354, 0.058856783978165, 0.168490940966118,
      0.228948642746032);
  Eigen::Quaterniond q2(
      0.960768653534504, 0.065511336552757, 0.177531610326744,
      0.202766827916600);
  Eigen::Quaterniond q3(
      0.957187982859276, 0.050483920758591, 0.190308650775464,
      0.212191320874942);

  Eigen::MatrixXd samples =
      common::RotationUtils::ConvertQuaternionsToMatrix({q1, q2, q3});
  Eigen::RowVector3d weights(0.2, 0.7, 0.1);

  common::Bingham bingham = common::Bingham::fit(samples, weights);
  Eigen::VectorXd true_mode(4);
  true_mode << 0.959724413089442, 0.062683254650737, 0.177016105317784,
      0.208962099349714;
  EXPECT_NEAR_EIGEN(bingham.mode(), true_mode, 1e-4);
}

TEST_F(BinghamTest, sampleDeterministicTest) {
  Eigen::MatrixXd M = Eigen::MatrixXd::Identity(2, 2);
  Eigen::Vector2d Z(-3, 0);
  common::Bingham bingham(Z, M);
  Eigen::MatrixXd deterministic_samples;
  Eigen::RowVectorXd deterministic_weights;
  bingham.sampleDeterministic(&deterministic_samples, &deterministic_weights);

  Eigen::MatrixXd true_samples(2, 3);
  Eigen::RowVector3d true_weights(0.3333333, 0.3333333, 0.3333333);
  true_samples << 0, 0.550363580623329, -0.550363580623329, 1,
      0.834925103900624, 0.834925103900624;

  EXPECT_NEAR_EIGEN(deterministic_weights, true_weights, 1e-4);
  EXPECT_NEAR_EIGEN(deterministic_samples, true_samples, 1e-4);
}

TEST_F(BinghamTest, fitSingleTest) {
  Eigen::Quaterniond q1(
      0.956937406927354, 0.058856783978165, 0.168490940966118,
      0.228948642746032);
  Eigen::MatrixXd samples =
      common::RotationUtils::ConvertQuaternionsToMatrix({q1});
  Eigen::RowVectorXd weights(1);
  weights << 1;

  common::Bingham bingham = common::Bingham::fit(samples, weights);
  Eigen::VectorXd true_mode(4);
  true_mode << 0.956937406927354, 0.058856783978165, 0.168490940966118,
      0.228948642746032;
  EXPECT_NEAR_EIGEN(bingham.mode(), true_mode, 1e-4);
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
