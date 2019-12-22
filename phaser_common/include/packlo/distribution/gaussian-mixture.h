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

  const Eigen::VectorXd& getMixtureMean() const;
  const Eigen::MatrixXd& getMixtureCov() const;

 private:
  void initializeUniformWeights();
  std::pair<Eigen::VectorXd, Eigen::MatrixXd> calcMixutreParameters();

  // Components.
  Eigen::MatrixXd means_;
  Eigen::MatrixXd cov;
  std::vector<Eigen::MatrixXd> covs_;
  Eigen::VectorXd weights_;

  Eigen::VectorXd mean_;
  Eigen::MatrixXd cov_;
};

}  // namespace common

#endif  // PACKLO_DISTRIBUTION_GAUSSIAN_MIXTURE_H_
