#include "phaser/distribution/bingham-mixture.h"

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

class BinghamMixtureTest : public ::testing::Test {
 public:
  BinghamMixtureTest() = default;

 protected:
  virtual void SetUp() {}
};

TEST_F(BinghamMixtureTest, constructorGloverTest) {
  Eigen::MatrixXd M = Eigen::MatrixXd::Identity(4, 4);
  Eigen::Vector4d Z(-10, -2, -1, 0);
  common::Bingham bingham(Z, M);
  common::Bingham bingham2(Z, M);
  Eigen::Vector2d weights(0.8, 0.2);
  std::vector<common::Bingham> binghams{bingham, bingham2};

  common::BinghamMixture bmm(binghams, weights);
  EXPECT_NEAR_EIGEN(bmm.getMixtureZ(), Z, 5);
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
