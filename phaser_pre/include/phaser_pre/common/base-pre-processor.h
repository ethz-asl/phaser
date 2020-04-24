#ifndef PHASER_PRE_COMMON_BASE_PRE_PROCESSOR_H_
#define PHASER_PRE_COMMON_BASE_PRE_PROCESSOR_H_

#include "phaser/model/point-cloud.h"

namespace preproc {

class BasePreProcessor {
 public:
  virtual void process(model::PointCloudPtr cloud) = 0;
};

}  // namespace preproc

#endif  // PHASER_PRE_COMMON_BASE_PRE_PROCESSOR_H_
