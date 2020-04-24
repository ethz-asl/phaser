#ifndef PHASER_PRE_MODEL_OCCLUSION_RESULT_H_
#define PHASER_PRE_MODEL_OCCLUSION_RESULT_H_

#include <vector>

#include "phaser_pre/common/vec-helper.h"

namespace preproc {

class OcclusionResult {
 public:
  OcclusionResult();

  std::vector<int>& getPickedNeighbors();
  const std::vector<int>& getPickedNeighbors() const;

 private:
  std::vector<int> neighbor_picked_;
  AlgorithmSettings settings_;
};

}  // namespace preproc

#endif  // PHASER_PRE_MODEL_OCCLUSION_RESULT_H_
