#include "packlo/model/state.h"

namespace model {

common::DualQuaternion State::getCurrentState() const {
  return common::DualQuaternion();
}

const common::Gaussian& State::getTranslationalDistribution() const {
  return trans_distribution_;
}

const common::Bingham& State::getRotationalDistribution() const {
  return rot_distribution_;
}

}  // namespace model
