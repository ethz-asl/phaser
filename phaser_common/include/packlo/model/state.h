#ifndef PACKLO_MODEL_STATE_H_
#define PACKLO_MODEL_STATE_H_

#include "packlo/common/dual-quaternion.h"
#include "packlo/distribution/bingham.h"
#include "packlo/distribution/gaussian.h"

namespace model {

class State {
 public:
  State() = default;

  common::DualQuaternion getCurrentState() const;

  void setRotationalDistribution(common::BaseDistributionPtr rot_dist);
  void setTranslationalDistribution(common::BaseDistributionPtr pos_dist);

  common::BaseDistributionPtr getRotationalDistribution() const;
  common::BaseDistributionPtr getTranslationalDistribution() const;

 private:
  common::BaseDistributionPtr rot_distribution_ = nullptr;
  common::BaseDistributionPtr trans_distribution_ = nullptr;
};

}  // namespace model

#endif  // PACKLO_MODEL_STATE_H_
