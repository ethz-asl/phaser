#ifndef PHASER_PRE_CLOUD_PRE_PROCESSOR_H_
#define PHASER_PRE_CLOUD_PRE_PROCESSOR_H_

#include <vector>
#include "phaser_pre/common/base-command.h"
#include "phaser_pre/common/base-pre-processor.h"

namespace preproc {

class CloudPreProcessor {
 public:
  CloudPreProcessor();

 private:
  std::vector<BaseCommandPtr> processors_;
};

}  // namespace preproc

#endif  // PHASER_PRE_CLOUD_PRE_PROCESSOR_H_
