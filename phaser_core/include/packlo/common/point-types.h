#ifndef INCLUDE_PACKLO_COMMON_POINT_TYPES_H_
#define INCLUDE_PACKLO_COMMON_POINT_TYPES_H_

#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>
#include <pcl/common/projection_matrix.h>
#include <Eigen/Dense>

namespace common {

using Point_t = pcl::PointXYZI;
using PointCloud_t = pcl::PointCloud<Point_t>;
using PointCloud_tPtr = pcl::PointCloud<Point_t>::Ptr;

using Vector_t = Eigen::Vector3d;
}  // namespace common

#endif  // INCLUDE_PACKLO_COMMON_POINT_TYPES_H_
