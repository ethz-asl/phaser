#include "phaser_pre/algorithm/angle-based-ground-removal.h"

#define USE_SSE2
#include "phaser_pre/common/sse-mathfun-extension.h"
#include "phaser_pre/common/vec-helper.h"

namespace preproc {

GroundRemovalResult AngleBasedGroundRemoval::removeGroundSeq(
    common::PointCloud_tPtr cloud) {
  cv::Mat ground_mat = cv::Mat(
      settings_.N_SCAN, settings_.Horizon_SCAN, CV_8S, cv::Scalar::all(0));
  for (std::size_t j = 0u; j < settings_.Horizon_SCAN; ++j) {
    removeGroundForIndex(cloud, 0, j, &ground_mat);
  }
  return GroundRemovalResult(std::move(ground_mat));
}

GroundRemovalResult AngleBasedGroundRemoval::removeGround(
    common::PointCloud_tPtr cloud) {
  cv::Mat ground_mat = cv::Mat(
      settings_.N_SCAN, settings_.Horizon_SCAN, CV_8S, cv::Scalar::all(0));
  std::size_t idx;
  const __m128 threshold = _mm_set1_ps(0.174533f);
  const __m128 mountAngle = _mm_set1_ps(settings_.sensorMountAngle);
  const uint32_t ground_scan_id_vec = settings_.groundScanInd - 3;
  for (std::size_t j = 0u; j < settings_.Horizon_SCAN; ++j) {
    std::size_t i = 0u;
    for (; i < ground_scan_id_vec; i += 4) {
      __m128 vecXY = _mm_setzero_ps();
      __m128 vecXY2 = _mm_setzero_ps();
      __m128 vecXYShifted = _mm_setzero_ps();
      __m128 vecXYShifted2 = _mm_setzero_ps();
      __m128 vecZ = _mm_setzero_ps();
      __m128 vecZShifted = _mm_setzero_ps();

      idx = j + (i)*settings_.Horizon_SCAN;
      vecXY[0] = cloud->points[idx].x;
      vecXY[1] = cloud->points[idx].y;
      vecZ[0] = cloud->points[idx].z;

      idx = j + (i + 1) * settings_.Horizon_SCAN;
      vecXY[2] = cloud->points[idx].x;
      vecXY[3] = cloud->points[idx].y;
      vecZ[1] = cloud->points[idx].z;
      vecXYShifted[0] = cloud->points[idx].x;
      vecXYShifted[1] = cloud->points[idx].y;
      vecZShifted[0] = cloud->points[idx].z;

      idx = j + (i + 2) * settings_.Horizon_SCAN;
      vecXY2[0] = cloud->points[idx].x;
      vecXY2[1] = cloud->points[idx].y;
      vecXYShifted[2] = cloud->points[idx].x;
      vecXYShifted[3] = cloud->points[idx].y;
      vecZShifted[1] = cloud->points[idx].z;

      idx = j + (i + 3) * settings_.Horizon_SCAN;
      vecXY2[2] = cloud->points[idx].x;
      vecXY2[3] = cloud->points[idx].y;
      vecZ[3] = cloud->points[idx].z;
      vecXYShifted2[0] = cloud->points[idx].x;
      vecXYShifted2[1] = cloud->points[idx].y;
      vecZShifted[2] = cloud->points[idx].z;

      idx = j + (i + 4) * settings_.Horizon_SCAN;
      vecXYShifted2[2] = cloud->points[idx].x;
      vecXYShifted2[3] = cloud->points[idx].y;
      vecZShifted[3] = cloud->points[idx].z;

      // Calculate the difference between lower and upper bound.
      const __m128 diff = _mm_sub_ps(vecXYShifted, vecXY);
      const __m128 diff2 = _mm_sub_ps(vecXYShifted2, vecXY2);

      // Squaring the difference.
      const __m128 diffSquared = _mm_mul_ps(diff, diff);
      const __m128 diffSquared2 = _mm_mul_ps(diff2, diff2);

      // Horizontal pairwise addition.
      const __m128 sum = _mm_hadd_ps(diffSquared, diffSquared2);

      // Calculate the absolute angle.
      const __m128 diffZ = _mm_sub_ps(vecZShifted, vecZ);
      const __m128 angleV = atan2_ps(diffZ, _mm_sqrt_ps(sum));
      const __m128 absAngle =
          _mm_andnot_ps(settings_.sign_mask, _mm_sub_ps(angleV, mountAngle));

      // Create a conditional variable.
      const __m128 ground = _mm_cmple_ps(absAngle, threshold);
      const uint32_t cond = _mm_movemask_epi8(_mm_castps_si128(ground));
      if (cond & 0x000F) {
        ground_mat.at<int8_t>(i, j) = 1;
        ground_mat.at<int8_t>(i + 1, j) = 1;
      }
      if (cond & 0x00F0) {
        ground_mat.at<int8_t>(i + 1, j) = 1;
        ground_mat.at<int8_t>(i + 2, j) = 1;
      }
      if (cond & 0x0F00) {
        ground_mat.at<int8_t>(i + 2, j) = 1;
        ground_mat.at<int8_t>(i + 3, j) = 1;
      }
      if (cond & 0xF000) {
        ground_mat.at<int8_t>(i + 3, j) = 1;
        ground_mat.at<int8_t>(i + 4, j) = 1;
      }
    }

    // Process the remaining points in the cloud sequentially.
    removeGroundForIndex(cloud, i, j, &ground_mat);
  }
  return GroundRemovalResult(std::move(ground_mat));
}

void AngleBasedGroundRemoval::removeGroundForIndex(
    common::PointCloud_tPtr cloud, const std::size_t start, const std::size_t j,
    cv::Mat* ground_mat) {
  for (std::size_t i = start; i < settings_.groundScanInd; ++i) {
    size_t lowerInd = j + (i)*settings_.Horizon_SCAN;
    size_t upperInd = j + (i + 1) * settings_.Horizon_SCAN;

    if (cloud->points[lowerInd].intensity == -1 ||
        cloud->points[upperInd].intensity == -1) {
      // no info to check, invalid points
      ground_mat->at<int8_t>(i, j) = -1;
      continue;
    }

    const float dX = cloud->points[upperInd].x - cloud->points[lowerInd].x;
    const float dY = cloud->points[upperInd].y - cloud->points[lowerInd].y;
    const float dZ = cloud->points[upperInd].z - cloud->points[lowerInd].z;

    const float vertical_angle =
        std::atan2(dZ, sqrt(dX * dX + dY * dY + dZ * dZ));

    if ((vertical_angle - settings_.sensorMountAngle) <= 0.174533f) {
      ground_mat->at<int8_t>(i, j) = 1;
      ground_mat->at<int8_t>(i + 1, j) = 1;
    }
  }
}
}  // namespace preproc
