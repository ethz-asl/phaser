#include "packlo/distribution/gaussian-mixture.h"

namespace common {

GaussianMixture::GaussianMixture(
    const std::vector<Eigen::VectorXd>& means,
    const std::vector<Eigen::MatrixXd>& covs)
    : means_(means), covs_(covs) {
  initializeUniformWeights();
}

GaussianMixture::GaussianMixture(
    const std::vector<Eigen::VectorXd>& means,
    const std::vector<Eigen::MatrixXd>& covs, const Eigen::VectorXd& weights)
    : means_(means), covs_(covs), weights_(weights) {}

void GaussianMixture::initializeUniformWeights() {
  const double n_dim = means_.size();
  weights_ = Eigen::VectorXd::Zero(n_dim);
  weights_.array() = 1 / n_dim;
}

}  // namespace common
