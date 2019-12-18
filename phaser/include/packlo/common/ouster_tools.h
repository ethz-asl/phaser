#pragma once

#include <packlo/common/os1_64_config.h>
#include <packlo/model/point.h>

#include <ouster/os1.h>
#include <ouster/os1_packet.h>


namespace common {

static Point nth_point(int ind, const uint8_t* col_buf) {
  float h_angle_0 = ouster::OS1::col_h_angle(col_buf);
  auto tte = pointcloud_assembler::trig_table[ind];
  const uint8_t* px_buf = ouster::OS1::nth_px(ind, col_buf);
  float r = ouster::OS1::px_range(px_buf) / 1000.0f;
  float h_angle = tte.beam_azimuth_angles + h_angle_0;

  model::PointXYZI point;
  point.x() = -r * tte.cos_beam_altitude_angles * std::cosf(h_angle);
  point.y() = r * tte.cos_beam_altitude_angles * std::sinf(h_angle);
  point.z() = r * tte.sin_beam_altitude_angles;
  point.i() = ouster::OS1::px_signal_photons(px_buf);
  
  /*
  point.reflectivity = ouster::OS1::px_reflectivity(px_buf);
  point.intensity = ouster::OS1::px_signal_photons(px_buf);
  point.x = -r * tte.cos_beam_altitude_angles * cosf(h_angle);
  point.y = r * tte.cos_beam_altitude_angles * sinf(h_angle);
  point.z = r * tte.sin_beam_altitude_angles;
  point.ring = ind;
  */

  return point;
}

} // namespace common
