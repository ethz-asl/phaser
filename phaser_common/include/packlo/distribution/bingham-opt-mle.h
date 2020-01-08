#ifndef PACKLO_DISTRIBUTION_BINGHAM_OPT_MLE_H_
#define PACKLO_DISTRIBUTION_BINGHAM_OPT_MLE_H_

#define _USE_MATH_DEFINES

#include <Eigen/Dense>

namespace common {

class BinghamOptMLE {
 public:
  static Eigen::VectorXd compute(const Eigen::VectorXd& omega);
  // static int compute(int dim, double* in, double* res);

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace common

#endif  // PACKLO_DISTRIBUTION_BINGHAM_OPT_MLE_H_
