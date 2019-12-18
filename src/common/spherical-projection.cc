#include "packlo/common/spherical-projection.h"

#include <glog/logging.h>
#include <cmath>

namespace common {

void SphericalProjection::convertPointCloud(model::PointCloud* cloud) {
  naiveProjection(*cloud, cloud);
}

model::PointCloud SphericalProjection::convertPointCloudCopy(
    const model::PointCloud &cloud) {
  model::PointCloud cloned = cloud.clone();
  naiveProjection(cloud, &cloned);
  return cloned;
}

void SphericalProjection::naiveProjection(
    const model::PointCloud& cloud_in, model::PointCloud* cloud_out) {
  const uint32_t n_points = cloud_in.size();
  for (uint32_t i = 0u; i < n_points; ++i) {
    const common::Point_t &point = cloud_in.pointAt(i);
    // Calculate the distance to the point.
    const double dist_xy = std::sqrt(point.x * point.x + point.y * point.y);
    const double dist = std::sqrt(dist_xy * dist_xy + point.z * point.z);

    // Set spherical coordinates.
    common::Point_t& point_out = cloud_out->pointAt(i);
    point_out.x = std::acos(point.z / dist);
    point_out.y = std::fmod(std::atan2(point.y, point.x) + 2*M_PI, 2*M_PI);

    // Convert back to cartesian coordinates.
    point_out.x = std::sin(point_out.x) * std::cos(point_out.y);
    point_out.y = std::sin(point_out.x) * std::sin(point_out.y);
    point_out.z = std::cos(point_out.x);
  }
}

}  // namespace common
