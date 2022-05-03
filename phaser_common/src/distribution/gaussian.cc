#include "phaser/distribution/gaussian.h"

#include <glog/logging.h>

namespace common {

Gaussian::Gaussian(const Eigen::VectorXd& mu, const Eigen::MatrixXd& cov)
    : mu_(mu), cov_(cov) {
  CHECK_EQ(mu_.rows(), 3);
  CHECK_EQ(cov_.rows(), 3);
  CHECK_EQ(cov_.cols(), 3);
}

Gaussian::Gaussian(
    const Eigen::MatrixXd& samples, const Eigen::VectorXd& weights) {
  CHECK_GT(samples.cols(), 0);
  CHECK_GT(samples.rows(), 0);
  CHECK_EQ(samples.cols(), weights.rows());
  setMeanAndCov(samples, weights);
  samples_ = samples;
  weights_ = weights;
}

Eigen::VectorXd Gaussian::getEstimate() const {
  return mu_;
}

Eigen::VectorXd& Gaussian::getMean() {
  return mu_;
}

const Eigen::VectorXd& Gaussian::getMean() const {
  return mu_;
}

Eigen::MatrixXd& Gaussian::getCov() {
  return cov_;
}

const Eigen::MatrixXd& Gaussian::getCov() const {
  return cov_;
}

std::pair<Eigen::VectorXd, Eigen::MatrixXd> Gaussian::getParameters() const {
  return std::make_pair(mu_, cov_);
}

void Gaussian::setMeanAndCov(
    const Eigen::MatrixXd& samples, const Eigen::VectorXd& weights) {
  mu_ = samples * weights;
  Eigen::MatrixXd zero_mean_samples = samples.colwise() - mu_;
  Eigen::MatrixXd sqrt_weights = weights.transpose().array().sqrt();
  Eigen::MatrixXd weighted_samples = zero_mean_samples.cwiseProduct(
      sqrt_weights.replicate(zero_mean_samples.rows(), 1));
  cov_ = weighted_samples * weighted_samples.transpose();
}

}  // namespace common
