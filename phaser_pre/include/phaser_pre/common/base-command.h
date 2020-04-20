#ifndef PHASER_PRE_COMMON_BASE_COMMAND_H_
#define PHASER_PRE_COMMON_BASE_COMMAND_H_

#include <memory>

#include "phaser/model/point-cloud.h"

namespace preproc {

class BaseCommand {
 public:
  virtual void execute(model::PointCloudPtr cloud) = 0;
};

using BaseCommandPtr = std::unique_ptr<BaseCommand>;

}  // namespace preproc

#endif  // PHASER_PRE_COMMON_BASE_COMMAND_H_
