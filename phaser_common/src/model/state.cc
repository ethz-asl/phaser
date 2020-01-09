#include "packlo/model/state.h"

namespace model {

common::DualQuaternion State::getCurrentState() const {
  return common::DualQuaternion();
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
