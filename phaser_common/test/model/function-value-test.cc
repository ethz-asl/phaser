#include "phaser/model/function-value.h"

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

class FunctionValueTest : public ::testing::Test {
 public:
  FunctionValueTest()
      : generator_(std::chrono::system_clock::now().time_since_epoch().count()),
        distribution_(0, 100) {}

  common::Point_t getRandomPoint() {
    common::Point_t p;
    p.x = getRandomFloat();
    p.y = getRandomFloat();
    p.z = getRandomFloat();
    return p;
  }

  float getRandomFloat() {
    return distribution_(generator_);
  }

 private:
  std::default_random_engine generator_;
  std::uniform_real_distribution<float> distribution_;
};

TEST_F(FunctionValueTest, SingleValueTest) {
  const float range = getRandomFloat();
  const float intensity = getRandomFloat();
  const common::Point_t p = getRandomPoint();
  model::FunctionValue f_val;
  f_val.addRange(range);
  f_val.addIntensity(intensity);
  f_val.addPoint(p);

  constexpr float tol = 0.001;
  EXPECT_NEAR(f_val.getAveragedRange(), range, tol);
  EXPECT_NEAR(f_val.getAveragedIntensity(), intensity, tol);
  const common::Point_t avg_p = f_val.getAveragedPoint();
  EXPECT_NEAR(avg_p.x, p.x, tol);
  EXPECT_NEAR(avg_p.y, p.y, tol);
  EXPECT_NEAR(avg_p.z, p.z, tol);
}

TEST_F(FunctionValueTest, AverageValueTest) {
  const float range = getRandomFloat();
  const float range2 = getRandomFloat();
  const float intensity = getRandomFloat();
  const float intensity2 = getRandomFloat();
  const common::Point_t p = getRandomPoint();
  const common::Point_t p2 = getRandomPoint();
  model::FunctionValue f_val;
  f_val.addRange(range);
  f_val.addRange(range2);
  f_val.addIntensity(intensity);
  f_val.addIntensity(intensity2);
  f_val.addPoint(p);
  f_val.addPoint(p2);

  constexpr float tol = 0.001;
  EXPECT_NEAR(f_val.getAveragedRange(), (range + range2) / 2.0f, tol);
  EXPECT_NEAR(
      f_val.getAveragedIntensity(), (intensity + intensity2) / 2.0f, tol);
  const common::Point_t avg_p = f_val.getAveragedPoint();
  EXPECT_NEAR(avg_p.x, (p.x + p2.x) / 2.0f, tol);
  EXPECT_NEAR(avg_p.y, (p.y + p2.y) / 2.0f, tol);
  EXPECT_NEAR(avg_p.z, (p.z + p2.z) / 2.0f, tol);
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
