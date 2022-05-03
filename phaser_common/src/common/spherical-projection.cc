#include "phaser/common/spherical-projection.h"

#include <cmath>
#include <glog/logging.h>

namespace common {

void SphericalProjection::convertPointCloud(model::PointCloud* cloud) {
  naiveProjection(*cloud, cloud);
}

model::PointCloud SphericalProjection::convertPointCloudCopy(
    const model::PointCloud& cloud) {
  model::PointCloud cloned = cloud.clone();
  naiveProjection(cloud, &cloned);
  return cloned;
}

void SphericalProjection::naiveProjection(
    const model::PointCloud& cloud_in, model::PointCloud* cloud_out) {
  const uint32_t n_points = cloud_in.size();
  for (uint32_t i = 0u; i < n_points; ++i) {
    const common::Point_t& point = cloud_in.pointAt(i);
    // Calculate the distance to the point.
    const double dist_xy = std::sqrt(point.x * point.x + point.y * point.y);
    const double dist = std::sqrt(dist_xy * dist_xy + point.z * point.z);

    // Set spherical coordinates.
    common::Point_t& point_out = cloud_out->pointAt(i);
    point_out.z = point.z / dist;
    cloud_out->setRange(dist, i);

    const double tmp_x = std::acos(point_out.z);
    const double tmp_y =
        std::fmod(std::atan2(point.y, point.x) + 2 * M_PI, 2 * M_PI);

    // Convert back to cartesian coordinates.
    point_out.x = std::sin(tmp_x) * std::cos(tmp_y);
    point_out.y = std::sin(tmp_x) * std::sin(tmp_y);
  }
}

}  // namespace common
