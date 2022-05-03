#include "phaser/backend/correlation/spatial-correlation.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <gtest/gtest.h>
#include <memory>
#include <random>

#include "phaser/common/signal-utils.h"
#include "phaser/common/test/testing-entrypoint.h"
#include "phaser/common/test/testing-predicates.h"
#include "phaser/common/translation-utils.h"

namespace phaser_core {

class SpatialCorrelationTest : public ::testing::Test {
 public:
  SpatialCorrelationTest()
      : generator_(std::chrono::system_clock::now().time_since_epoch().count()),
        distribution_(0, 100) {}

 protected:
  float getRandomFloat() {
    return distribution_(generator_);
  }
  uint32_t nnz(double* signal, const uint32_t n_coeff) {
    CHECK_NOTNULL(signal);
    uint32_t zeros = 0u;
    for (uint32_t i = 0u; i < n_coeff; ++i) {
      if (abs(signal[i]) > 0) {
        ++zeros;
      }
    }
    return zeros;
  }

  std::default_random_engine generator_;
  std::uniform_real_distribution<float> distribution_;
};

TEST_F(SpatialCorrelationTest, SimpleCorrelation) {
  const uint32_t n_corr = 27u;
  Eigen::VectorXd f = Eigen::VectorXd::Zero(n_corr);
  Eigen::VectorXd g = Eigen::VectorXd::Zero(n_corr);
  for (uint32_t i = 0; i < n_corr; ++i) {
    f(i) = getRandomFloat();
    g(i) = getRandomFloat();
  }
  SpatialCorrelation corr(3u);
  std::vector<Eigen::VectorXd*> signal_f = {&f};
  std::vector<Eigen::VectorXd*> signal_g = {&g};
  double* c = corr.correlateSignals(signal_f, signal_g);

  CHECK_NOTNULL(c);
  EXPECT_GT(nnz(c, n_corr), 0);
}

TEST_F(SpatialCorrelationTest, CorrelationEndToEnd) {
  const uint32_t n_corr = 27u;
  Eigen::VectorXd f = Eigen::VectorXd::Zero(n_corr);
  Eigen::VectorXd g = Eigen::VectorXd::Zero(n_corr);
  f(0) = 1;
  g(1) = 1;
  SpatialCorrelation corr(3u);
  std::vector<Eigen::VectorXd*> signal_f = {&f};
  std::vector<Eigen::VectorXd*> signal_g = {&g};
  double* c = corr.correlateSignals(signal_f, signal_g);
  CHECK_NOTNULL(c);

  const std::vector<double> c_vec(c, c + n_corr);
  auto max_it = std::max_element(c_vec.cbegin(), c_vec.cend());
  const uint32_t lin_idx_max = std::distance(c_vec.cbegin(), max_it);
  const std::array<uint32_t, 3> xyz =
      common::SignalUtils::Ind2Sub(lin_idx_max, 3, 3);
  EXPECT_EQ(xyz[0], 0u);
  EXPECT_EQ(xyz[1], 2u);
  EXPECT_EQ(xyz[2], 0u);
}

TEST_F(SpatialCorrelationTest, SimplePaddedCorrelation) {
  const uint32_t n_corr_per_dim = 3u;
  const uint32_t n_corr = n_corr_per_dim * n_corr_per_dim * n_corr_per_dim;
  Eigen::VectorXd f = Eigen::VectorXd::Zero(n_corr);
  Eigen::VectorXd g = Eigen::VectorXd::Zero(n_corr);
  for (uint32_t i = 0; i < n_corr; ++i) {
    f(i) = getRandomFloat();
    g(i) = getRandomFloat();
  }
  const uint32_t padding = 2u;
  SpatialCorrelation corr(n_corr_per_dim, padding);
  std::vector<Eigen::VectorXd*> signal_f = {&f};
  std::vector<Eigen::VectorXd*> signal_g = {&g};
  double* c = corr.correlateSignals(signal_f, signal_g);

  CHECK_NOTNULL(c);
  const uint32_t n_padded_corr_per_dim = n_corr_per_dim + 2 * padding;
  const uint32_t n_padded_cor =
      n_padded_corr_per_dim * n_padded_corr_per_dim * n_padded_corr_per_dim;
  EXPECT_GT(nnz(c, n_padded_cor), 0);
}

TEST_F(SpatialCorrelationTest, PaddedCorrelation) {
  const uint32_t n_corr_per_dim = 3u;
  const uint32_t n_corr = n_corr_per_dim * n_corr_per_dim * n_corr_per_dim;
  Eigen::VectorXd f = Eigen::VectorXd::Zero(n_corr);
  Eigen::VectorXd g = Eigen::VectorXd::Zero(n_corr);
  f(0) = 1.0;
  g(1) = 1.0;

  const uint32_t padding = 2u;
  SpatialCorrelation corr(n_corr_per_dim, padding);
  std::vector<Eigen::VectorXd*> signal_f = {&f};
  std::vector<Eigen::VectorXd*> signal_g = {&g};
  double* c = corr.correlateSignals(signal_f, signal_g);
  CHECK_NOTNULL(c);

  const uint32_t n_padded_corr_per_dim = n_corr_per_dim + 2 * padding;
  const uint32_t n_padded_cor =
      n_padded_corr_per_dim * n_padded_corr_per_dim * n_padded_corr_per_dim;
  const std::vector<double> c_vec(c, c + n_padded_cor);
  auto max_it = std::max_element(c_vec.cbegin(), c_vec.cend());
  const uint32_t lin_idx_max = std::distance(c_vec.cbegin(), max_it) + 1;
  const std::array<uint32_t, 3> xyz = common::SignalUtils::Ind2Sub(
      lin_idx_max, n_padded_corr_per_dim, n_padded_corr_per_dim);
  double x = common::TranslationUtils::ComputeTranslationFromIndex(
      static_cast<double>(xyz[0]), n_padded_corr_per_dim, 0, 3);
  double y = common::TranslationUtils::ComputeTranslationFromIndex(
      static_cast<double>(xyz[1]), n_padded_corr_per_dim, 0, 3);
  double z = common::TranslationUtils::ComputeTranslationFromIndex(
      static_cast<double>(xyz[2]), n_padded_corr_per_dim, 0, 3);
  EXPECT_EQ(x, 0u);
  EXPECT_NEAR(y, -1.0, 0.2);
  EXPECT_EQ(z, 0u);
}

}  // namespace phaser_core

MAPLAB_UNITTEST_ENTRYPOINT
