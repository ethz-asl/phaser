#include "phaser/distribution/bingham-mixture.h"

#include <glog/logging.h>

DEFINE_int32(
    bmm_sample_size, 70,
    "Determines the number of samples used for the BMM calculation.");

namespace common {

BinghamMixture::BinghamMixture(
    const std::vector<Bingham>& binghams, const Eigen::VectorXd& weights)
    : binghams_(binghams), weights_(weights) {
  CHECK_EQ(binghams_.size(), weights_.rows());
  CHECK_GT(binghams_.size(), 0);

  // Verify that all distributions have the same dimensions.
  dim_ = binghams_.front().getDim();
  for (const common::Bingham& bingham : binghams_) {
    CHECK_EQ(dim_, bingham.getDim());
  }

  bingham_sample_size_ = FLAGS_bmm_sample_size;
  calcMixtureParametersGlover();
}

Eigen::VectorXd BinghamMixture::getEstimate() const {
  return mode_;
}

const Eigen::VectorXd& BinghamMixture::getMixtureMode() const {
  return mode_;
}

const Eigen::MatrixXd& BinghamMixture::getMixtureS2() const {
  return S2_;
}

const Eigen::VectorXd& BinghamMixture::getMixtureZ() const {
  return Z_;
}

const Eigen::MatrixXd& BinghamMixture::getMixtureM() const {
  return M_;
}

uint16_t BinghamMixture::getSampleSize() const noexcept {
  return bingham_sample_size_;
}

uint16_t& BinghamMixture::getSampleSize() {
  return bingham_sample_size_;
}

void BinghamMixture::calcMixtureParametersGlover() {
  const uint16_t n_binghams = binghams_.size();
  Eigen::MatrixXd samples(dim_, n_binghams * bingham_sample_size_);
  Eigen::RowVectorXd weights(n_binghams * bingham_sample_size_);
  Eigen::RowVectorXd ones = Eigen::VectorXd::Ones(bingham_sample_size_);
  for (uint16_t i = 0u; i < n_binghams; ++i) {
    const Bingham& bingham = binghams_[i];
    const double weight = weights_[i];
    // todo(lbern): use deterministic or glover sampling?
    Eigen::MatrixXd local_samples;
    bingham.sampleGlover(&local_samples, bingham_sample_size_);

    // Update the samples and weights
    const uint16_t col_idx = i * bingham_sample_size_;
    samples.block(0, col_idx, dim_, bingham_sample_size_) = local_samples;
    weights.block(0, col_idx, 1, bingham_sample_size_) = ones.array() * weight;
  }
  weights = weights.array() / weights.array().sum();
  const common::Bingham fitted = common::Bingham::fit(samples, weights);
  Z_ = fitted.getZ();
  M_ = fitted.getM();
  mode_ = fitted.mode();
  S2_ = fitted.moment();
}

}  // namespace common
