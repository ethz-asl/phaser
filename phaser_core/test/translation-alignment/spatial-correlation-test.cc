#include "phaser/backend/correlation/spatial-correlation.h"
#include "phaser/common/test/testing-entrypoint.h"
#include "phaser/common/test/testing-predicates.h"

#include <chrono>
#include <cmath>
#include <gtest/gtest.h>
#include <memory>
#include <random>

namespace translation {

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
  correlation::SpatialCorrelation corr(3u);
  std::vector<Eigen::VectorXd*> signal_f = {&f};
  std::vector<Eigen::VectorXd*> signal_g = {&g};
  double* c = corr.correlateSignals(signal_f, signal_g);

  CHECK_NOTNULL(c);
  EXPECT_GT(nnz(c, n_corr), 0);
}

}  // namespace translation

MAPLAB_UNITTEST_ENTRYPOINT
