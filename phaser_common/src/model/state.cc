#include "phaser/model/state.h"

#include <glog/logging.h>

#include "phaser/distribution/bingham.h"
#include "phaser/distribution/gaussian.h"

namespace model {

common::DualQuaternion State::getCurrentState() const {
  Eigen::Vector4d b_est(1, 0, 0, 0);
  if (rot_distribution_ != nullptr)
    b_est = rot_distribution_->getEstimate();
  Eigen::Vector3d g_est(0, 0, 0);
  if (trans_distribution_ != nullptr)
    g_est = trans_distribution_->getEstimate().block(0, 0, 3, 1);
  Eigen::VectorXd dq(8);
  dq << b_est, 0, g_est;
  return common::DualQuaternion(dq);
}

void State::setRotationalDistribution(common::BaseDistributionPtr rot_dist) {
  rot_distribution_ = rot_dist;
}

void State::setTranslationalDistribution(common::BaseDistributionPtr pos_dist) {
  trans_distribution_ = pos_dist;
}

common::BaseDistributionPtr State::getRotationalDistribution() const {
  return rot_distribution_;
}

common::BaseDistributionPtr State::getTranslationalDistribution() const {
  return trans_distribution_;
}

}  // namespace model
