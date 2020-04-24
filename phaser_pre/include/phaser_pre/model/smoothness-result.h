#ifndef PHASER_PRE_MODEL_SMOOTHNESS_RESULT_H_
#define PHASER_PRE_MODEL_SMOOTHNESS_RESULT_H_

#include <utility>
#include <vector>

#include "phaser_pre/common/vec-helper.h"

namespace preproc {

using smoothness_t = std::pair<float, std::size_t>;

class SmoothnessResult {
 public:
  SmoothnessResult();
  std::vector<smoothness_t>& getSmoothness();
  const std::vector<smoothness_t>& getSmoothness() const;

 private:
  std::vector<smoothness_t> smoothness_;
  AlgorithmSettings settings_;
};

}  // namespace preproc

#endif  // PHASER_PRE_MODEL_SMOOTHNESS_RESULT_H_
