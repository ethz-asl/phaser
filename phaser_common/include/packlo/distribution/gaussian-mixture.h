#ifndef PACKLO_DISTRIBUTION_GAUSSIAN_MIXTURE_H_
#define PACKLO_DISTRIBUTION_GAUSSIAN_MIXTURE_H_

#include <Eigen/Dense>
#include <utility>
#include <vector>

namespace common {

class GaussianMixture {
 public:
  explicit GaussianMixture(
      const Eigen::MatrixXd& means, const std::vector<Eigen::MatrixXd>& covs);
  explicit GaussianMixture(
      const Eigen::MatrixXd& means, const std::vector<Eigen::MatrixXd>& covs,
      const Eigen::VectorXd& weights);

 private:
  void initializeUniformWeights();
  std::pair<Eigen::VectorXd, Eigen::MatrixXd> calcMixutreParameters();

  // Components.
  Eigen::VectorXd means_;
  Eigen::MatrixXd cov;
  std::vector<Eigen::MatrixXd> covs_;
  Eigen::VectorXd weights_;

  Eigen::VectorXd mean_;
  Eigen::MatrixXd cov_;
};

}  // namespace common

#endif  // PACKLO_DISTRIBUTION_GAUSSIAN_MIXTURE_H_
