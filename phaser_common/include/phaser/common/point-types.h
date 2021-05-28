#ifndef PHASER_COMMON_POINT_TYPES_H_
#define PHASER_COMMON_POINT_TYPES_H_

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

namespace common {

using Point_t = pcl::PointXYZI;
using PointCloud_t = pcl::PointCloud<Point_t>;
using PointCloud_tPtr = pcl::PointCloud<Point_t>::Ptr;

using Vector_t = Eigen::Vector3d;
}  // namespace common

#endif  // PHASER_COMMON_POINT_TYPES_H_
