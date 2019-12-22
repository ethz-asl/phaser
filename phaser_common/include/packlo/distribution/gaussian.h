#ifndef GAUSSIAN_H_
#define GAUSSIAN_H_

#include <Eigen/Dense>
#include "packlo/distribution/base-distribution.h"

namespace common {

class Gaussian : public BaseDistribution {
 public:
  explicit(Eigen::VectorXd mu, Eigen::MatrixXd C);
}

}  // namespace common

#endif  // GAUSSIAN_H_
