#ifndef PHASER_DISTRIBUTION_BINGHAM_OBJECTIVE_H_
#define PHASER_DISTRIBUTION_BINGHAM_OBJECTIVE_H_

#define _USE_MATH_DEFINES

#include <Eigen/Dense>
#include <vector>

namespace common {

class BinghamObjective {
 public:
  explicit BinghamObjective(const Eigen::Vector4d& omega);

  double optimize(const std::vector<double>& x);

 public:
  const Eigen::Vector4d& omega_;
};

}  // namespace common

#endif  // PHASER_DISTRIBUTION_BINGHAM_OBJECTIVE_H_
