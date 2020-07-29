#ifndef PHASER_PRE_COMMON_VEC_HELPER_H_
#define PHASER_PRE_COMMON_VEC_HELPER_H_

#include <cmath>
#include <emmintrin.h>
#include <pmmintrin.h>
#include <smmintrin.h>
#include <xmmintrin.h>

// ToDo(lbern): make this proper

namespace preproc {

struct AlgorithmSettings {
  AlgorithmSettings() {
    rad2degConv = _mm_div_ps(_mm_set_ps1(180.0f), _mm_set_ps1(M_PI));
  }

  const int N_SCAN = 128;
  const int Horizon_SCAN = 2000;
  const float ang_res_x = 0.18;
  const float ang_res_y = 0.52;
  const float ang_bottom = 46.7;
  const int groundScanInd = 25;
  const float sensorMountAngle = 0.0 * 180 / M_PI;
  const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
  const float segmentAlphaY = ang_res_y / 180.0 * M_PI;
  const float segmentTheta = 20.0 / 180.0 * M_PI;

  __m128 rad2degConv = _mm_div_ps(_mm_set_ps1(180.0f), _mm_set_ps1(M_PI));

  const __m128 ang_bottom_ps = _mm_set_ps1(ang_bottom);
  const __m128 ang_res_x_ps = _mm_set_ps1(ang_res_x);
  const __m128 ang_res_y_ps = _mm_set_ps1(ang_res_y);
  const __m128 n_scan_ps = _mm_set_ps1(N_SCAN);
  const __m128 horizon_scan_ps = _mm_set_ps1(Horizon_SCAN);
  const __m128 horizon_scan_half_ps = _mm_set_ps1(Horizon_SCAN / 2.0f);
  const __m128 degree90_ps = _mm_set_ps1(90.0f);

  const __m128 zero_ps = _mm_set_ps1(0.0f);
  const __m128 tenThousand_ps = _mm_set_ps1(10000.0f);
  const __m128 sign_mask = _mm_set1_ps(-0.f);

  __m128 rad2deg_ps(__m128 radian) {
    return _mm_mul_ps(radian, rad2degConv);
  }
};

}  // namespace preproc

#endif  // PHASER_PRE_COMMON_VEC_HELPER_H_
