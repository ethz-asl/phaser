#ifndef PHASER_DISTRIBUTION_GAUSSIAN_H_
#define PHASER_DISTRIBUTION_GAUSSIAN_H_

#include <Eigen/Dense>
#include <memory>
#include <utility>

#include "phaser/distribution/base-distribution.h"

namespace common {

class Gaussian : public BaseDistribution {
 public:
  explicit Gaussian(const Eigen::VectorXd& mu, const Eigen::MatrixXd& cov);
  explicit Gaussian(
      const Eigen::MatrixXd& samples, const Eigen::VectorXd& weights);
  virtual ~Gaussian() = default;
  Eigen::VectorXd getEstimate() const override;

  Eigen::VectorXd& getMean();
  const Eigen::VectorXd& getMean() const;

  Eigen::MatrixXd& getCov();
  const Eigen::MatrixXd& getCov() const;

  std::pair<Eigen::VectorXd, Eigen::MatrixXd> getParameters() const;

  Eigen::MatrixXd samples_;
  Eigen::VectorXd weights_;

 private:
  void setMeanAndCov(
      const Eigen::MatrixXd& samples, const Eigen::VectorXd& weights);

  Eigen::VectorXd mu_;
  Eigen::MatrixXd cov_;
};

using GaussianPtr = std::shared_ptr<Gaussian>;

}  // namespace common

#endif  // PHASER_DISTRIBUTION_GAUSSIAN_H_
