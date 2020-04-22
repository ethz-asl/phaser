#ifndef PHASER_PRE_ALGORITHM_MARK_OCCLUDED_H_
#define PHASER_PRE_ALGORITHM_MARK_OCCLUDED_H_

#include "phaser_pre/model/occlusion-result.h"
#include "phaser_pre/model/segmentation-result.h"

namespace preproc {

class MarkOccluded {
 public:
  OcclusionResult compute(const SegmentationResult& seg_result);
};

}  // namespace preproc

#endif  // PHASER_PRE_ALGORITHM_MARK_OCCLUDED_H_
