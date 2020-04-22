#ifndef PHASER_PRE_ALGORITHM_CLOUD_SEGMENTATION_H_
#define PHASER_PRE_ALGORITHM_CLOUD_SEGMENTATION_H_

#include "phaser_pre/model/cluster-result.h"
#include "phaser_pre/model/projection-result.h"
#include "phaser_pre/model/segmentation-result.h"

namespace preproc {

class CloudSegmentation {
 public:
  void segment();
};

}  // namespace preproc

#endif  // PHASER_PRE_ALGORITHM_CLOUD_SEGMENTATION_H_
