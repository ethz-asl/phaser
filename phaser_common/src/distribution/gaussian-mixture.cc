#include "phaser/distribution/gaussian-mixture.h"

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

GaussianMixture::GaussianMixture(
    const std::vector<Gaussian>& gaussians, const Eigen::VectorXd& weights)
    : means_(Eigen::MatrixXd::Zero(3, gaussians.size())), weights_(weights) {
  setMeansAndCovsFromGaussians(gaussians);
  std::tie(mean_, cov_) = calcMixutreParameters();
}

Eigen::VectorXd GaussianMixture::getEstimate() const {
  return mean_;
}

void GaussianMixture::initializeFromGaussians(
    const std::vector<Gaussian>& gaussians, const Eigen::VectorXd& weights) {
  means_ = Eigen::MatrixXd::Zero(3, gaussians.size());
  weights_ = weights;
  setMeansAndCovsFromGaussians(gaussians);
  std::tie(mean_, cov_) = calcMixutreParameters();
}

void GaussianMixture::initializeUniformWeights() {
  const double n_dim = means_.size();
  weights_ = Eigen::VectorXd::Zero(n_dim);
  weights_.array() = 1 / n_dim;
}

void GaussianMixture::setMeansAndCovsFromGaussians(
    const std::vector<Gaussian>& gaussians) {
  const uint16_t n_gaussians = gaussians.size();
  CHECK_EQ(means_.rows(), 3);
  CHECK_EQ(means_.cols(), n_gaussians);
  for (uint16_t i = 0; i < n_gaussians; ++i) {
    means_.col(i) = gaussians.at(i).getMean();
    covs_.emplace_back(gaussians.at(i).getCov());
  }
}

std::pair<Eigen::VectorXd, Eigen::MatrixXd>
GaussianMixture::calcMixutreParameters() {
  CHECK_NE(means_.rows(), 0);
  CHECK_EQ(means_.cols(), covs_.size());
  CHECK_EQ(means_.cols(), weights_.rows());

  // Compute sample mean and covariance.
  Gaussian mean_Gaussian(means_, weights_);
  Eigen::VectorXd sample_mean = mean_Gaussian.getMean();
  Eigen::MatrixXd cov_mean = mean_Gaussian.getCov();

  // Combine the covariances with the sample covariance.
  Eigen::MatrixXd sum_cov =
      Eigen::MatrixXd::Zero(cov_mean.rows(), cov_mean.cols());
  const std::size_t n_samples = means_.cols();
  for (std::size_t i = 0u; i < n_samples; ++i) {
    sum_cov += covs_[i] * weights_(i);
  }

  Eigen::MatrixXd cov = sum_cov + cov_mean;
  return std::make_pair(std::move(sample_mean), std::move(cov));
}

const Eigen::VectorXd& GaussianMixture::getMixtureMean() const {
  return mean_;
}

const Eigen::MatrixXd& GaussianMixture::getMixtureCov() const {
  return cov_;
}

}  // namespace common
