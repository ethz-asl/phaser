#include "phaser/common/signal-utils.h"

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <fftw3/fftw3.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <memory>
#include <random>

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
  std::vector<fftw_complex> shifted(n_points);
  std::vector<fftw_complex> original(n_points);
  for (std::size_t i = 0; i < n_points; ++i) {
    const float val = getRandomFloat();
    shifted[i][0] = val;
    original[i][0] = val;
    shifted[i][1] = 0;
    original[i][1] = 0;
  }
  SignalUtils::FFTShift(shifted.data(), n_points);

  constexpr float tol = 0.001;
  const std::size_t n_points_remaining = n_points - n_points_half;
  for (std::size_t i = 0; i < n_points_remaining; ++i) {
    EXPECT_NEAR(original[i][0], shifted[i + n_points_half][0], tol);
    EXPECT_NEAR(original[i][1], shifted[i + n_points_half][1], tol);
  }
  for (std::size_t i = 0; i < n_points_half; ++i) {
    EXPECT_NEAR(original[i + n_points_half + 1][0], shifted[i][0], tol);
    EXPECT_NEAR(original[i + n_points_half + 1][1], shifted[i][1], tol);
  }
}

TEST_F(SignalUtilsTest, FFTShiftComplexEvenTest) {
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

TEST_F(SignalUtilsTest, IFFTShiftDoubleEvenTest) {
  const std::size_t n_points = 10;
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

TEST_F(SignalUtilsTest, IFFTShiftComplexEvenTest) {
  const std::size_t n_points = 10;
  std::vector<fftw_complex> shifted(n_points);
  std::vector<fftw_complex> original(n_points);
  for (std::size_t i = 0; i < n_points; ++i) {
    const float val = getRandomFloat();
    const float val2 = getRandomFloat();
    shifted[i][0] = val;
    original[i][0] = val;
    shifted[i][1] = val2;
    original[i][1] = val2;
  }
  constexpr float tol = 0.001;
  EXPECT_NEAR(original[0][0], shifted[0][0], tol);
  SignalUtils::FFTShift(shifted.data(), n_points);
  EXPECT_FALSE(std::abs(original[0][0] - shifted[0][0]) < tol);
  SignalUtils::IFFTShift(shifted.data(), n_points);

  for (std::size_t i = 0; i < n_points; ++i) {
    EXPECT_NEAR(original[i][0], shifted[i][0], tol);
    EXPECT_NEAR(original[i][1], shifted[i][1], tol);
  }
}

TEST_F(SignalUtilsTest, FFTShiftComplexSeqEvenTest) {
  const std::size_t n_points = 10;
  const std::size_t n_points_half = n_points / 2;
  std::vector<fftw_complex> shifted(n_points);
  std::vector<fftw_complex> original(n_points);
  for (std::size_t i = 0; i < n_points; ++i) {
    const float val = static_cast<float>(i + 1);
    const float val2 = static_cast<float>(i + 2);
    shifted[i][0] = val;
    original[i][0] = val;
    shifted[i][1] = val2;
    original[i][1] = val2;
  }
  constexpr float tol = 0.001;
  EXPECT_NEAR(original[0][0], shifted[0][0], tol);
  SignalUtils::FFTShift(shifted.data(), n_points);

  for (std::size_t i = 0; i < n_points_half; ++i) {
    const std::size_t shifted_i = n_points_half + i;
    EXPECT_NEAR(original[i][0], shifted[shifted_i][0], tol);
    EXPECT_NEAR(original[i][1], shifted[shifted_i][1], tol);
  }
  EXPECT_FALSE(std::abs(original[0][0] - shifted[0][0]) < tol);
  SignalUtils::IFFTShift(shifted.data(), n_points);
}

TEST_F(SignalUtilsTest, IFFTShiftComplexSeqEvenTest) {
  const std::size_t n_points = 10;
  const std::size_t n_points_half = n_points / 2;
  std::vector<fftw_complex> shifted(n_points);
  std::vector<fftw_complex> original(n_points);
  for (std::size_t i = 0; i < n_points; ++i) {
    const float val = static_cast<float>(i + 1);
    const float val2 = static_cast<float>(i + 2);
    shifted[i][0] = val;
    original[i][0] = val;
    shifted[i][1] = val2;
    original[i][1] = val2;
  }
  constexpr float tol = 0.001;
  SignalUtils::IFFTShift(shifted.data(), n_points);

  for (std::size_t i = 0; i < n_points_half; ++i) {
    const std::size_t shifted_i = n_points_half + i;
    EXPECT_NEAR(original[i][0], shifted[shifted_i][0], tol);
    EXPECT_NEAR(original[i][1], shifted[shifted_i][1], tol);
  }
}

TEST_F(SignalUtilsTest, Sub2IndTest) {
  const uint32_t n_voxels = 201;
  EXPECT_EQ(SignalUtils::Sub2Ind(100, 100, 100, n_voxels, n_voxels), 4060300);
  EXPECT_EQ(SignalUtils::Sub2Ind(120, 120, 120, n_voxels, n_voxels), 4872360);
  EXPECT_EQ(SignalUtils::Sub2Ind(80, 80, 80, n_voxels, n_voxels), 3248240);
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
