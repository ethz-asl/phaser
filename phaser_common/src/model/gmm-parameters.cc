#include "phaser/model/gmm-parameters.h"

namespace model {

GmmParameters::GmmParameters(
    Eigen::MatrixXd&& means, std::vector<Eigen::MatrixXd>&& covs,
    Eigen::VectorXd&& weights)
    : means_(means), covs_(covs), weights_(weights) {}

const Eigen::MatrixXd& GmmParameters::getMeans() const {
  return means_;
}

Eigen::MatrixXd& GmmParameters::getMeans() {
  return means_;
}

const std::vector<Eigen::MatrixXd>& GmmParameters::getCovs() const {
  return covs_;
}

std::vector<Eigen::MatrixXd>& GmmParameters::getCovs() {
  return covs_;
}

const Eigen::VectorXd& GmmParameters::getWeights() const {
  return weights_;
}

Eigen::VectorXd& GmmParameters::getWeights() {
  return weights_;
}

}  // namespace model
