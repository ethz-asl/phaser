#pragma once

#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

//namespace common {

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
//}
