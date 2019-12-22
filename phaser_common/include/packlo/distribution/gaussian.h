#ifndef PACKLO_DISTRIBUTION_GAUSSIAN_H_
#define PACKLO_DISTRIBUTION_GAUSSIAN_H_

#include <Eigen/Dense>
#include <utility>
#include "packlo/distribution/base-distribution.h"

namespace common {

class Gaussian : public BaseDistribution {
 public:
  explicit Gaussian(Eigen::VectorXd mu, Eigen::MatrixXd cov);

  Eigen::VectorXd& getMean();
  const Eigen::VectorXd& getMean() const;

  Eigen::MatrixXd& getCov();
  const Eigen::MatrixXd& getCov() const;

  std::pair<Eigen::VectorXd, Eigen::MatrixXd> getParameters() const;

 private:
  Eigen::VectorXd mu_;
  Eigen::MatrixXd cov_;
};

}  // namespace common

#endif  // PACKLO_DISTRIBUTION_GAUSSIAN_H_
