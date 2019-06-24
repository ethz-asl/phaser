#include <packlo/common/spherical-projection.h>

#include <glog/logging.h>

namespace common {

void SphericalProjection::convertPointCloud(model::PointCloud &cloud) {
  naiveProjection(cloud);
}


void SphericalProjection::naiveProjection(model::PointCloud &cloud) {
  for (model::Point_t point : cloud ) {
  }
}

}
