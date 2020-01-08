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

  const common::Gaussian& getTranslationalDistribution() const;
  const common::Bingham& getRotationalDistribution() const;

 private:
  common::Gaussian trans_distribution_;
  common::Bingham rot_distribution_;
};

}  // namespace model

#endif  // PACKLO_MODEL_STATE_H_
