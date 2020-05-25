#include "phaser/common/core-gflags.h"

namespace phaser_core {

// Spherical correlation.
DEFINE_int32(
    phaser_core_spherical_bandwidth, 75, "Defines the spherical bandwith.");
DEFINE_int32(
    phaser_core_spherical_zero_padding, 0,
    "Specifies whether and how much zero padding should be applied to the "
    "spherical correlation.");

// Spatial correlation.
DEFINE_int32(
    phaser_core_spatial_n_voxels, 100,
    "Defines the number of voxels used per dimension"
    " in the spatial correlation.");
DEFINE_int32(
    phaser_core_spatial_discretize_lower, -50,
    "Specifies the lower bound for the discretization.");
DEFINE_int32(
    phaser_core_spatial_discretize_upper, 50,
    "Specifies the upper bound for the discretization.");
DEFINE_int32(
    phaser_core_spatial_zero_padding, 0,
    "Specifies whether the spatial correlation should make use of zero "
    "padding.");

}  // namespace phaser_core
