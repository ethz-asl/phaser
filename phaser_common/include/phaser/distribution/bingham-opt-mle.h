#ifndef PHASER_DISTRIBUTION_BINGHAM_OPT_MLE_H_
#define PHASER_DISTRIBUTION_BINGHAM_OPT_MLE_H_

#define _USE_MATH_DEFINES

#include <Eigen/Dense>

namespace common {

class BinghamOptMLE {
 public:
  static Eigen::VectorXd compute(const Eigen::VectorXd& omega);

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace common

#endif  // PHASER_DISTRIBUTION_BINGHAM_OPT_MLE_H_
