#include "packlo/model/state.h"

#include <glog/logging.h>

namespace model {

common::DualQuaternion State::getCurrentState() const {
  Eigen::VectorXd b_est = rot_distribution_->getEstimate();
  Eigen::VectorXd g_est = trans_distribution_->getEstimate();
  Eigen::VectorXd dq(8);
  dq << b_est, 0, g_est.block(0, 0, 3, 1);
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
