#include <chrono>
#include <cmath>
#include <memory>
#include <random>

#include <Eigen/Dense>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "phaser/common/signal-utils.h"
#include "phaser/common/test/testing-entrypoint.h"
#include "phaser/common/test/testing-predicates.h"

namespace common {

class SignalUtilsTest : public ::testing::Test {
 public:
  SignalUtilsTest()
      : generator_(std::chrono::system_clock::now().time_since_epoch().count()),
        distribution_(0, 100) {}

  float getRandomFloat() {
    return distribution_(generator_);
  }

 private:
  std::default_random_engine generator_;
  std::uniform_real_distribution<float> distribution_;
};

TEST_F(SignalUtilsTest, FFTShiftDoubleEvenTest) {
  const std::size_t n_points = 10;
  const std::size_t n_points_half = n_points / 2;
  std::vector<double> shifted(n_points);
  std::vector<double> original(n_points);
  for (std::size_t i = 0; i < n_points; ++i) {
    const float val = getRandomFloat();
    shifted[i] = val;
    original[i] = val;
  }
  SignalUtils::FFTShift(shifted.data(), n_points);

  constexpr float tol = 0.001;
  for (std::size_t i = 0; i < n_points_half; ++i) {
    EXPECT_NEAR(original[i], shifted[i + n_points_half], tol);
    EXPECT_NEAR(original[i + n_points_half], shifted[i], tol);
  }
}

TEST_F(SignalUtilsTest, FFTShiftDoubleOddTest) {
  const std::size_t n_points = 7;
  const std::size_t n_points_half = floor(n_points / 2.0);
  std::vector<double> shifted(n_points);
  std::vector<double> original(n_points);
  for (std::size_t i = 0; i < n_points; ++i) {
    const float val = getRandomFloat();
    shifted[i] = val;
    original[i] = val;
  }
  SignalUtils::FFTShift(shifted.data(), n_points);

  constexpr float tol = 0.001;
  const std::size_t n_points_remaining = n_points - n_points_half;
  for (std::size_t i = 0; i < n_points_remaining; ++i) {
    EXPECT_NEAR(original[i], shifted[i + n_points_half], tol);
  }
  for (std::size_t i = 0; i < n_points_half; ++i) {
    EXPECT_NEAR(original[i + n_points_half + 1], shifted[i], tol);
  }
}

TEST_F(SignalUtilsTest, IFFTShiftDoubleEvenTest) {
  const std::size_t n_points = 10;
  const std::size_t n_points_half = n_points / 2;
  std::vector<double> shifted(n_points);
  std::vector<double> original(n_points);
  for (std::size_t i = 0; i < n_points; ++i) {
    const float val = getRandomFloat();
    shifted[i] = val;
    original[i] = val;
  }
  constexpr float tol = 0.001;
  EXPECT_NEAR(original[0], shifted[0], tol);
  SignalUtils::FFTShift(shifted.data(), n_points);
  EXPECT_FALSE(std::abs(original[0] - shifted[0]) < tol);
  SignalUtils::IFFTShift(shifted.data(), n_points);

  for (std::size_t i = 0; i < n_points; ++i) {
    EXPECT_NEAR(original[i], shifted[i], tol);
  }
}

TEST_F(SignalUtilsTest, IFFTShiftDoubleOddTest) {
  const std::size_t n_points = 7;
  const std::size_t n_points_half = floor(n_points / 2.0);
  std::vector<double> shifted(n_points);
  std::vector<double> original(n_points);
  for (std::size_t i = 0; i < n_points; ++i) {
    const float val = getRandomFloat();
    shifted[i] = val;
    original[i] = val;
  }

  constexpr float tol = 0.001;
  EXPECT_NEAR(original[0], shifted[0], tol);
  SignalUtils::FFTShift(shifted.data(), n_points);
  EXPECT_FALSE(std::abs(original[0] - shifted[0]) < tol);
  SignalUtils::IFFTShift(shifted.data(), n_points);

  for (std::size_t i = 0; i < n_points; ++i) {
    EXPECT_NEAR(original[i], shifted[i], tol);
  }
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
