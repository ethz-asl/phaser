#ifndef PHASER_PRE_ALGORITHM_CALC_SMOOTHNESS_H_
#define PHASER_PRE_ALGORITHM_CALC_SMOOTHNESS_H_

#include "phaser_pre/common/vec-helper.h"
#include "phaser_pre/model/segmentation-result.h"
#include "phaser_pre/model/smoothness-result.h"

namespace preproc {

class CalcSmoothness {
 public:
  SmoothnessResult compute(const SegmentationResult& seg_result);

  AlgorithmSettings settings_;
};

}  // namespace preproc

#endif  // PHASER_PRE_ALGORITHM_CALC_SMOOTHNESS_H_
