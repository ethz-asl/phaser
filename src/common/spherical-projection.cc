#include <packlo/common/spherical-projection.h>

#include <glog/logging.h>

#include <cmath>

namespace common {

void SphericalProjection::convertPointCloud(model::PointCloud &cloud) {
  naiveProjection(cloud);
}


void SphericalProjection::naiveProjection(model::PointCloud &cloud) {
  for (model::Point_t &point : cloud ) {
    // calculate the distance to the point.
    float dist_xy = std::sqrt(point.x * point.x + point.y*point.y);
    float dist = std::sqrt(dist_xy * dist_xy + point.z*point.z);

    // set spherical coordinates.
    point.x = std::acos(point.z / dist);
    point.y = std::fmod(std::atan2(point.y, point.x) + 2*M_PI, 2*M_PI);
    point.z = dist;
  }
}

}
