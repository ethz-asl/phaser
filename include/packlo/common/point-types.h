#ifndef INCLUDE_PACKLO_COMMON_POINT_TYPES_H_
#define INCLUDE_PACKLO_COMMON_POINT_TYPES_H_

#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>
#include <pcl/common/projection_matrix.h>
#include <Eigen/Dense>

// This has to be in global namespace
// otherwise PCL complains.

struct OusterPointType {
  PCL_ADD_POINT4D
  int time_offset_us;
  uint16_t reflectivity;
  uint16_t signal;
  uint8_t ring;
  float intensity;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointType,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (int, time_offset_us, time_offset_us)
    (uint16_t, reflectivity, reflectivity)
    (uint16_t, signal, intensity)
    (uint8_t, ring, ring))

struct SemanticKITTIPointType {
  PCL_ADD_POINT4D
  float semantic;
  float instance;
  float reflectivity;
  float intensity;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    SemanticKITTIPointType,
    (float, x,
     x)(float, y,
        y)(float, z, z)(float, semantic, semantic)(float, instance, instance)(
        float, reflectivity, reflectivity)(float, intensity, intensity))

namespace common {
// using Point_t = ::OusterPointType;
using Point_t = pcl::PointXYZI;
// using Point_t = ::SemanticKITTIPointType;
using PointCloud_t = pcl::PointCloud<Point_t>;
using PointCloud_tPtr = pcl::PointCloud<Point_t>::Ptr;
using Vector_t = Eigen::Vector3d;
}

#endif  // INCLUDE_PACKLO_COMMON_POINT_TYPES_H_
