#include "packlo/distribution/gaussian.h"

namespace common {

Gaussian::Gaussian(const Eigen::VectorXd& mu, const Eigen::MatrixXd& cov)
    : mu_(mu), cov_(cov) {}

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

}  // namespace common
