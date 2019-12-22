#include "packlo/distribution/gaussian-mixture.h"

#include <glog/logging.h>

namespace common {

GaussianMixture::GaussianMixture(
    const Eigen::MatrixXd& means, const std::vector<Eigen::MatrixXd>& covs)
    : means_(means), covs_(covs) {
  initializeUniformWeights();
  std::tie(mean_, cov_) = calcMixutreParameters();
}

GaussianMixture::GaussianMixture(
    const Eigen::MatrixXd& means, const std::vector<Eigen::MatrixXd>& covs,
    const Eigen::VectorXd& weights)
    : means_(means), covs_(covs), weights_(weights) {
  std::tie(mean_, cov_) = calcMixutreParameters();
}

void GaussianMixture::initializeUniformWeights() {
  const double n_dim = means_.size();
  weights_ = Eigen::VectorXd::Zero(n_dim);
  weights_.array() = 1 / n_dim;
}

std::pair<Eigen::VectorXd, Eigen::MatrixXd>
GaussianMixture::calcMixutreParameters() {
  CHECK(means_.rows() != 0);
  CHECK_EQ(means_.cols(), covs_.size());
  CHECK_EQ(means_.cols(), weights_.rows());
  // Compute sample mean and covariance.
  Eigen::VectorXd sample_mean = means_ * weights_;
  const std::size_t n_samples = means_.cols();
  Eigen::MatrixXd weighted_zero_means =
      Eigen::MatrixXd::Zero(means_.rows(), means_.cols());
  for (std::size_t i = 0u; i < n_samples; ++i) {
    Eigen::VectorXd zero_mean_sample = means_.col(i) - sample_mean;
    weighted_zero_means.col(i) = zero_mean_sample * std::sqrt(weights_(i));
  }
  Eigen::MatrixXd cov_mean =
      weighted_zero_means * weighted_zero_means.transpose();

  // Combine the covariances with the sample covariance.
  Eigen::MatrixXd sum_cov =
      Eigen::MatrixXd::Zero(cov_mean.rows(), cov_mean.cols());
  for (std::size_t i = 0u; i < n_samples; ++i) {
    sum_cov += covs_[i] * weights_(i);
  }
  Eigen::MatrixXd cov = sum_cov + cov_mean;

  return std::make_pair<Eigen::VectorXd, Eigen::MatrixXd>(
      std::move(sample_mean), std::move(cov));
}

}  // namespace common
