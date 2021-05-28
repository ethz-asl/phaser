#ifndef PHASER_MODEL_GMM_PARAMETERS_H_
#define PHASER_MODEL_GMM_PARAMETERS_H_

#include <Eigen/Dense>
#include <vector>

namespace model {

class GmmParameters {
 public:
  GmmParameters() = default;
  explicit GmmParameters(
      Eigen::MatrixXd&& means, std::vector<Eigen::MatrixXd>&& covs,
      Eigen::VectorXd&& weights);

  const Eigen::MatrixXd& getMeans() const;
  Eigen::MatrixXd& getMeans();

  const std::vector<Eigen::MatrixXd>& getCovs() const;
  std::vector<Eigen::MatrixXd>& getCovs();

  const Eigen::VectorXd& getWeights() const;
  Eigen::VectorXd& getWeights();

 private:
  Eigen::MatrixXd means_;
  std::vector<Eigen::MatrixXd> covs_;
  Eigen::VectorXd weights_;
};

}  // namespace model

#endif  // PHASER_MODEL_GMM_PARAMETERS_H_
