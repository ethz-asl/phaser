#ifndef PHASER_PRE_ALGORITHM_EXTRACT_LOAM_FEATURES_H_
#define PHASER_PRE_ALGORITHM_EXTRACT_LOAM_FEATURES_H_

#include "phaser_pre/common/vec-helper.h"
#include "phaser_pre/model/feature-extraction-result.h"
#include "phaser_pre/model/occlusion-result.h"
#include "phaser_pre/model/segmentation-result.h"
#include "phaser_pre/model/smoothness-result.h"

namespace preproc {

class ExtractLoamFeatures {
 public:
  FeatureExtractionResult extractFeatures(
      const SegmentationResult& seg_result,
      const SmoothnessResult& smooth_result, const OcclusionResult& occ_result);

 private:
  AlgorithmSettings settings_;
};

}  // namespace preproc

#endif  // PHASER_PRE_ALGORITHM_EXTRACT_LOAM_FEATURES_H_
