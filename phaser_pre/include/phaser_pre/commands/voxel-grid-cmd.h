#ifndef PHASER_PRE_COMMANDS_VOXEL_GRID_CMD_H_
#define PHASER_PRE_COMMANDS_VOXEL_GRID_CMD_H_

#include "phaser_pre/common/base-command.h"

namespace preproc {

class VoxelGridCmd : public BaseCommand {
 public:
  void execute() override;
};

}  // namespace preproc

#endif  // PHASER_PRE_COMMANDS_VOXEL_GRID_CMD_H_
