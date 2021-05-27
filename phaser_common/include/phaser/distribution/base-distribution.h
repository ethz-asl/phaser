#ifndef PHASER_DISTRIBUTION_BASE_DISTRIBUTION_H_
#define PHASER_DISTRIBUTION_BASE_DISTRIBUTION_H_

#include <Eigen/Dense>
#include <memory>

namespace common {

class BaseDistribution {
 public:
  virtual ~BaseDistribution() = default;

  virtual Eigen::VectorXd getEstimate() const = 0;
};

using BaseDistributionPtr = std::shared_ptr<BaseDistribution>;

}  // namespace common

#endif  // PHASER_DISTRIBUTION_BASE_DISTRIBUTION_H_
