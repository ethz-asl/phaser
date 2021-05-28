/*
 * Bingham distritbution estimation based on maximum likelihood from samples.
 *
 * Reference goes to:
 * Igor Gilitschenski, Gerhard Kurz, Simon J. Julier, Uwe D. Hanebeck,
 * Efficient Bingham Filtering based on Saddlepoint Approximations
 * Proceedings of the 2014 IEEE International Conference on Multisensor Fusion
 * and Information Integration (MFI 2014), Beijing, China, September 2014.

 * Reference implementation:
 * Kurz et al., "Directional Statistics and Filtering Using libDirectional",
 * Journal of Statistical Software, vol. 89, no. 4 (2019): 1-31.
 */

#ifndef PHASER_DISTRIBUTION_BINGHAM_MLE_H_
#define PHASER_DISTRIBUTION_BINGHAM_MLE_H_

#define _USE_MATH_DEFINES

#include <Eigen/Dense>

class BinghamMLE {
 public:
  static Eigen::VectorXd compute(Eigen::VectorXd* omega);
  static int compute(int dim, double* in, double* res);

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif  // PHASER_DISTRIBUTION_BINGHAM_MLE_H_
