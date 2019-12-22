#ifndef PACKLO_DISTRIBUTION_GAUSSIAN_MIXTURE_H_
#define PACKLO_DISTRIBUTION_GAUSSIAN_MIXTURE_H_

#include <Eigen/Dense>
#include <vector>

namespace common {

class GaussianMixture {
 public:
  explicit GaussianMixture(
      const std::vector<Eigen::VectorXd>& means,
      const std::vector<Eigen::MatrixXd>& covs);
  explicit GaussianMixture(
      const std::vector<Eigen::VectorXd>& means,
      const std::vector<Eigen::MatrixXd>& covs, const Eigen::VectorXd& weights);

 private:
  void initializeUniformWeights();

  std::vector<Eigen::VectorXd> means_;
  std::vector<Eigen::MatrixXd> covs_;
  Eigen::VectorXd weights_;
};

}  // namespace common

#endif  // PACKLO_DISTRIBUTION_GAUSSIAN_MIXTURE_H_
