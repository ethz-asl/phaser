#include "phaser_pre/model/smoothness-result.h"

namespace preproc {

SmoothnessResult::SmoothnessResult() {
  smoothness_.resize(settings_.N_SCAN * settings_.Horizon_SCAN);
}

std::vector<smoothness_t>& SmoothnessResult::getSmoothness() {
  return smoothness_;
}

const std::vector<smoothness_t>& SmoothnessResult::getSmoothness() const {
  return smoothness_;
}

}  // namespace preproc
