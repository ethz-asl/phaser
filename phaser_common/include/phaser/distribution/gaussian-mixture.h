#ifndef PHASER_DISTRIBUTION_GAUSSIAN_MIXTURE_H_
#define PHASER_DISTRIBUTION_GAUSSIAN_MIXTURE_H_

#include <Eigen/Dense>
#include <memory>
#include <utility>
#include <vector>

#include "phaser/distribution/base-distribution.h"
#include "phaser/distribution/gaussian.h"

namespace common {

class GaussianMixture : public BaseDistribution {
 public:
  GaussianMixture() = default;
  explicit GaussianMixture(
      const Eigen::MatrixXd& means, const std::vector<Eigen::MatrixXd>& covs);
  explicit GaussianMixture(
      const Eigen::MatrixXd& means, const std::vector<Eigen::MatrixXd>& covs,
      const Eigen::VectorXd& weights);
  explicit GaussianMixture(
      const std::vector<Gaussian>& gaussians, const Eigen::VectorXd& weights);
  virtual ~GaussianMixture() = default;

  Eigen::VectorXd getEstimate() const override;

  void initializeFromGaussians(
      const std::vector<Gaussian>& gaussians, const Eigen::VectorXd& weights);

  const Eigen::VectorXd& getMixtureMean() const;
  const Eigen::MatrixXd& getMixtureCov() const;

 private:
  void initializeUniformWeights();
  void setMeansAndCovsFromGaussians(const std::vector<Gaussian>& gaussians);
  std::pair<Eigen::VectorXd, Eigen::MatrixXd> calcMixutreParameters();

  // Components.
  Eigen::MatrixXd means_;
  Eigen::MatrixXd cov;
  std::vector<Eigen::MatrixXd> covs_;
  Eigen::VectorXd weights_;

  Eigen::VectorXd mean_;
  Eigen::MatrixXd cov_;
};

using GaussianMixturePtr = std::shared_ptr<GaussianMixture>;

}  // namespace common

#endif  // PHASER_DISTRIBUTION_GAUSSIAN_MIXTURE_H_
