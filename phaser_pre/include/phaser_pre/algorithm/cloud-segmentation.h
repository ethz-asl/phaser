#ifndef PHASER_PRE_ALGORITHM_CLOUD_SEGMENTATION_H_
#define PHASER_PRE_ALGORITHM_CLOUD_SEGMENTATION_H_

#include "phaser_pre/common/vec-helper.h"
#include "phaser_pre/model/cluster-result.h"
#include "phaser_pre/model/ground-removal-result.h"
#include "phaser_pre/model/projection-result.h"
#include "phaser_pre/model/segmentation-result.h"

namespace preproc {

class CloudSegmentation {
 public:
  SegmentationResult segment(
      const ProjectionResult& proj_result, const ClusterResult& cluster_result,
      const GroundRemovalResult& ground_result);

 private:
  AlgorithmSettings settings_;
};

}  // namespace preproc

#endif  // PHASER_PRE_ALGORITHM_CLOUD_SEGMENTATION_H_
