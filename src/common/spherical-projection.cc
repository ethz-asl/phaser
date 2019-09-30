#include <packlo/common/spherical-projection.h>

#include <glog/logging.h>

#include <cmath>

namespace common {

void SphericalProjection::convertPointCloud(model::PointCloud &cloud) {
  naiveProjection(cloud, cloud);
}

model::PointCloud SphericalProjection::convertPointCloudCopy(
    const model::PointCloud &cloud) {
  model::PointCloud cloned = cloud.clone();
  naiveProjection(cloud, cloned);
  return cloned;
}

void SphericalProjection::naiveProjection(
    const model::PointCloud &cloud_in, model::PointCloud &cloud_out) {
  std::size_t n_points = cloud_in.size();
  for (std::size_t i = 0u; i < n_points; ++i) {
    const common::Point_t &point = cloud_in.pointAt(i);
    // calculate the distance to the point.
    float dist_xy = std::sqrt(point.x * point.x + point.y*point.y);
    float dist = std::sqrt(dist_xy * dist_xy + point.z*point.z);

    // set spherical coordinates.
    common::Point_t& point_out = cloud_out.pointAt(i);
    point_out.x = std::acos(point.z / dist);
    point_out.y = std::fmod(std::atan2(point.y, point.x) + 2*M_PI, 2*M_PI);
    point_out.z = 1;
  }
}

}
