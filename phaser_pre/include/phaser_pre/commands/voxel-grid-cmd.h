#ifndef PHASER_PRE_COMMANDS_VOXEL_GRID_CMD_H_
#define PHASER_PRE_COMMANDS_VOXEL_GRID_CMD_H_

#include <pcl/filters/voxel_grid.h>

#include "phaser_pre/common/base-command.h"

namespace preproc {

class VoxelGridCmd : public BaseCommand {
 public:
  void execute(model::PointCloudPtr cloud) override;

 private:
  pcl::VoxelGrid<common::Point_t> voxel_grid_filter_;
};

}  // namespace preproc

#endif  // PHASER_PRE_COMMANDS_VOXEL_GRID_CMD_H_
