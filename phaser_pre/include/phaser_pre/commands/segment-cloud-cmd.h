#ifndef PHASER_PRE_COMMANDS_SEGMENT_CLOUD_CMD_H_
#define PHASER_PRE_COMMANDS_SEGMENT_CLOUD_CMD_H_

#include <pcl/filters/voxel_grid.h>

#include "phaser_pre/algorithm/angle-based-ground-removal.h"
#include "phaser_pre/algorithm/cloud-segmentation.h"
#include "phaser_pre/algorithm/cluster-points.h"
#include "phaser_pre/algorithm/image-projection.h"
#include "phaser_pre/common/base-command.h"

namespace preproc {

class SegmentCloudCmd : public BaseCommand {
 public:
  void execute(model::PointCloudPtr cloud) override;

 private:
  ImageProjection proj_;
  ClusterPoints cluster_;
  AngleBasedGroundRemoval gnd_removal_;
  CloudSegmentation seg_;
};

}  // namespace preproc

#endif  // PHASER_PRE_COMMANDS_SEGMENT_CLOUD_CMD_H_
