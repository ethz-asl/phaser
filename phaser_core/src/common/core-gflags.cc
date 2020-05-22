#include "phaser/common/core-gflags.h"

namespace common {

DEFINE_int32(
    phaser_core_spherical_bandwidth, 75, "Defines the spherical bandwith.");
DEFINE_int32(
    phaser_core_spherical_zero_padding, 0,
    "Specifies whether and how much zero padding should be applied to the "
    "spherical correlation.");

}  // namespace common
