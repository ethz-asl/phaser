#ifndef PHASER_COMMON_CORE_GFLAGS_H_
#define PHASER_COMMON_CORE_GFLAGS_H_

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace common {

DECLARE_int32(phaser_core_spherical_bandwidth);
DECLARE_int32(phaser_core_spherical_zero_padding);

}  // namespace common

#endif  // PHASER_COMMON_CORE_GFLAGS_H_
