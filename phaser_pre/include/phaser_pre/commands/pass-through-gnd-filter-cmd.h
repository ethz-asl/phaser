#ifndef PHASER_PRE_COMMANDS_PASS_THROUGH_GND_FILTER_CMD_H_
#define PHASER_PRE_COMMANDS_PASS_THROUGH_GND_FILTER_CMD_H_

#include <pcl/filters/passthrough.h>

#include "phaser_pre/common/base-command.h"

namespace preproc {

class PassThroughGndFilterCmd : public BaseCommand {
 public:
  void execute(model::PointCloudPtr cloud) override;

 private:
  pcl::PassThrough<common::Point_t> gnd_filter_;
};

}  // namespace preproc

#endif  // PHASER_PRE_COMMANDS_PASS_THROUGH_GND_FILTER_CMD_H_
