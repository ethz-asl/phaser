#include "packlo/model/state.h"

#include <glog/logging.h>

namespace model {

common::DualQuaternion State::getCurrentState() const {
  return common::DualQuaternion();
}

void State::setRotationalDistribution(common::BaseDistributionPtr rot_dist) {
  rot_distribution_ = rot_dist;
}

void State::setTranslationalDistribution(common::BaseDistributionPtr pos_dist) {
  if (pos_dist != nullptr)
    VLOG(1) << "setting pos dist";
  else
    VLOG(1) << "setting null pos dist";
  trans_distribution_ = pos_dist;
}

common::BaseDistributionPtr State::getRotationalDistribution() const {
  return rot_distribution_;
}

common::BaseDistributionPtr State::getTranslationalDistribution() const {
  if (trans_distribution_ != nullptr)
    VLOG(1) << "NOT NULL";
  else
    VLOG(1) << " NULL";
  return trans_distribution_;
}

}  // namespace model
