#include "phaser_pre/model/occlusion-result.h"

namespace preproc {

OcclusionResult::OcclusionResult() {
  neighbor_picked_.assign(settings_.N_SCAN * settings_.Horizon_SCAN, 0);
}

std::vector<int>& OcclusionResult::getPickedNeighbors() {
  return neighbor_picked_;
}

const std::vector<int>& OcclusionResult::getPickedNeighbors() const {
  return neighbor_picked_;
}

}  // namespace preproc
