#ifndef INCLUDE_PACKLO_COMMON_SPHERICAL_PROJECTION_H_
#define INCLUDE_PACKLO_COMMON_SPHERICAL_PROJECTION_H_

#include "phaser/model/point-cloud.h"

namespace common {

class SphericalProjection {
 public:
  void convertPointCloud(model::PointCloud* cloud);
  model::PointCloud convertPointCloudCopy(const model::PointCloud& cloud);

 private:
  void naiveProjection(
      const model::PointCloud& cloud_in, model::PointCloud* cloud_out);
};

}  // namespace common

#endif  // INCLUDE_PACKLO_COMMON_SPHERICAL_PROJECTION_H_
