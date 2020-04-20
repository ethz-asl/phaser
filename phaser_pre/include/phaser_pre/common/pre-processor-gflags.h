#ifndef PHASER_PRE_PRE_PROCESSOR_GFLAGS_H_
#define PHASER_PRE_PRE_PROCESSOR_GFLAGS_H_

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace preproc {

DECLARE_bool(phaser_pre_enable_voxel_grid_downsampling);

}  // namespace preproc

#endif  // PHASER_PRE_PRE_PROCESSOR_GFLAGS_H_
