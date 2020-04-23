#include "phaser_pre/common/pre-processor-gflags.h"

namespace preproc {

DEFINE_bool(
    phaser_pre_enable_voxel_grid_downsampling, false,
    "Enable downsampling with a voxel grid.");

DEFINE_bool(
    phaser_pre_enable_pass_through_gnd_filtering, false,
    "Enable GND removal using pass-through filtering.");

DEFINE_bool(
    phaser_pre_enable_geometric_cloud_segmentation, false,
    "Enable geometric segmentation of the cloud.");

}  // namespace preproc
