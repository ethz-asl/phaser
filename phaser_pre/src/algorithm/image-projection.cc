#include "phaser_pre/algorithm/image-projection.h"

#include <chrono>
#include <glog/logging.h>

#define USE_SSE2
#include "phaser_pre/common/sse-mathfun-extension.h"
#include "phaser_pre/common/vec-helper.h"

namespace preproc {

ImageProjection::ImageProjection() {}

ProjectionResult ImageProjection::projectPointCloudSequential(
    model::PointCloudPtr cloud) {
  const std::size_t n_points = cloud->size();
  common::PointCloud_tPtr input_cloud = cloud->getRawCloud();
  common::PointCloud_tPtr full_cloud(new common::PointCloud_t);
  common::PointCloud_tPtr full_info_cloud(new common::PointCloud_t);

  AlgorithmSettings settings;
  const auto full_cloud_size = settings.N_SCAN * settings.Horizon_SCAN;
  full_cloud->points.resize(full_cloud_size);
  full_info_cloud->points.resize(full_cloud_size);
  cv::Mat range_mat = cv::Mat(
      settings.N_SCAN, settings.Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
  cv::Mat signal_mat = cv::Mat(
      settings.N_SCAN, settings.Horizon_SCAN, CV_32F, cv::Scalar::all(0));

  projectPointCloudSequentialImpl(
      input_cloud, 0, n_points, full_cloud, full_info_cloud, &range_mat,
      &signal_mat);
  return ProjectionResult(full_cloud, full_info_cloud, range_mat, signal_mat);
}

ProjectionResult ImageProjection::projectPointCloud(
    model::PointCloudPtr cloud) {
  common::PointCloud_tPtr full_cloud(new common::PointCloud_t);
  common::PointCloud_tPtr full_info_cloud(new common::PointCloud_t);
  AlgorithmSettings settings;
  full_cloud->points.resize(settings.N_SCAN * settings.Horizon_SCAN);
  full_info_cloud->points.resize(settings.N_SCAN * settings.Horizon_SCAN);
  cv::Mat range_mat = cv::Mat(
      settings.N_SCAN, settings.Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
  cv::Mat signal_mat = cv::Mat(
      settings.N_SCAN, settings.Horizon_SCAN, CV_32F, cv::Scalar::all(0));

  const std::size_t n_points = cloud->size();
  const std::size_t n_points_vec = n_points - 3;
  __m128 lowerVecXY = _mm_setzero_ps();
  __m128 upperVecXY = _mm_setzero_ps();
  __m128 vecX = _mm_setzero_ps();
  __m128 vecY = _mm_setzero_ps();
  __m128 vecZ = _mm_setzero_ps();
  __m128 verticalAngle = _mm_setzero_ps();
  __m128 horizonAngle = _mm_setzero_ps();
  __m128 sumSquaredXY = _mm_setzero_ps();
  __m128 rowIdn = _mm_setzero_ps();
  __m128 columnIdn = _mm_setzero_ps();
  __m128 range = _mm_setzero_ps();
  __m128 largerThanHorizon = _mm_setzero_ps();
  __m128 validRow = _mm_setzero_ps();
  __m128 intensity = _mm_setzero_ps();
  __m128 index = _mm_setzero_ps();
  uint32_t cond = 0;
  std::size_t i = 0;
  common::PointCloud_tPtr input_cloud = cloud->getRawCloud();

  for (; i < n_points_vec; i += 4) {
    const auto& point1 = input_cloud->points[i];
    const auto& point2 = input_cloud->points[i + 1];
    const auto& point3 = input_cloud->points[i + 2];
    const auto& point4 = input_cloud->points[i + 3];
    lowerVecXY[0] = point1.x;
    lowerVecXY[1] = point1.y;
    lowerVecXY[2] = point2.x;
    lowerVecXY[3] = point2.y;
    upperVecXY[0] = point3.x;
    upperVecXY[1] = point3.y;
    upperVecXY[2] = point4.x;
    upperVecXY[3] = point4.y;
    vecX[0] = point1.x;
    vecX[1] = point2.x;
    vecX[2] = point3.x;
    vecX[3] = point4.x;
    vecY[0] = point1.y;
    vecY[1] = point2.y;
    vecY[2] = point3.y;
    vecY[3] = point4.y;
    vecZ[0] = point1.z;
    vecZ[1] = point2.z;
    vecZ[2] = point3.z;
    vecZ[3] = point4.z;

    // Compute the vertical angle alpha, i.e.
    // alpha = atan2(z, sqrt(x^2 + y^2)).
    sumSquaredXY = _mm_hadd_ps(
        _mm_mul_ps(lowerVecXY, lowerVecXY), _mm_mul_ps(upperVecXY, upperVecXY));
    verticalAngle =
        settings.rad2deg_ps(atan2_ps(vecZ, _mm_sqrt_ps(sumSquaredXY)));

    rowIdn = _mm_round_ps(
        _mm_div_ps(
            _mm_add_ps(verticalAngle, settings.ang_bottom_ps),
            settings.ang_res_y_ps),
        _MM_FROUND_TO_ZERO);

    horizonAngle = settings.rad2deg_ps(atan2_ps(vecX, vecY));
    columnIdn = _mm_sub_ps(
        settings.horizon_scan_half_ps,
        _mm_round_ps(
            _mm_div_ps(
                _mm_sub_ps(horizonAngle, settings.degree90_ps),
                settings.ang_res_x_ps),
            _MM_FROUND_TO_ZERO));

    // columnIdn = columnIdn >= Horizon_SCAN ?
    //   columnIdn - Horizon_SCAN : columnIdn
    largerThanHorizon = _mm_cmpge_ps(columnIdn, settings.horizon_scan_ps);
    columnIdn = _mm_blendv_ps(
        columnIdn, _mm_sub_ps(columnIdn, settings.horizon_scan_ps),
        largerThanHorizon);

    range = _mm_sqrt_ps(_mm_add_ps(sumSquaredXY, _mm_mul_ps(vecZ, vecZ)));
    intensity =
        _mm_add_ps(rowIdn, _mm_div_ps(columnIdn, settings.tenThousand_ps));

    validRow = _mm_and_ps(
        _mm_cmpge_ps(rowIdn, settings.zero_ps),
        _mm_cmplt_ps(rowIdn, settings.n_scan_ps));
    cond = _mm_movemask_epi8(_mm_castps_si128(validRow));
    index = _mm_add_ps(columnIdn, _mm_mul_ps(rowIdn, settings.horizon_scan_ps));

    if (cond & 0x000F) {
      range_mat.at<float>(rowIdn[0], columnIdn[0]) = range[0];
      signal_mat.at<float>(rowIdn[0], columnIdn[0]) = point1.intensity;
      full_cloud->points[index[0]] = point1;
      // full_cloud->points[index[0]].intensity = intensity[0];
      full_info_cloud->points[index[0]].x = range[0];
      full_info_cloud->points[index[0]].y = point1.intensity;
    }
    if (cond & 0x00F0) {
      range_mat.at<float>(rowIdn[1], columnIdn[1]) = range[1];
      signal_mat.at<float>(rowIdn[1], columnIdn[1]) = point2.intensity;
      full_cloud->points[index[1]] = point2;
      // full_cloud->points[index[1]].intensity = intensity[1];
      full_info_cloud->points[index[1]].x = range[1];
      full_info_cloud->points[index[1]].y = point2.intensity;
    }
    if (cond & 0x0F00) {
      range_mat.at<float>(rowIdn[2], columnIdn[2]) = range[2];
      signal_mat.at<float>(rowIdn[2], columnIdn[2]) = point3.intensity;
      full_cloud->points[index[2]] = point3;
      // full_cloud->points[index[2]].intensity = intensity[2];
      full_info_cloud->points[index[2]].x = range[2];
      full_info_cloud->points[index[2]].y = point3.intensity;
    }
    if (cond & 0xF000) {
      range_mat.at<float>(rowIdn[3], columnIdn[3]) = range[3];
      signal_mat.at<float>(rowIdn[3], columnIdn[3]) = point4.intensity;
      full_cloud->points[index[3]] = point4;
      // full_cloud->points[index[3]].intensity = intensity[3];
      full_info_cloud->points[index[3]].intensity = range[3];
      full_info_cloud->points[index[3]].y = point4.intensity;
    }
  }

  // Process the remaining points in the cloud sequentially.
  projectPointCloudSequentialImpl(
      input_cloud, i, n_points, full_cloud, full_info_cloud, &range_mat,
      &signal_mat);
  return ProjectionResult(full_cloud, full_info_cloud, range_mat, signal_mat);
}

void ImageProjection::projectPointCloudSequentialImpl(
    common::PointCloud_tPtr cloud, const std::size_t start,
    const std::size_t end, common::PointCloud_tPtr full_cloud,
    common::PointCloud_tPtr full_info_cloud, cv::Mat* range_mat,
    cv::Mat* signal_mat) {
  CHECK_NOTNULL(cloud);
  CHECK_NOTNULL(full_cloud);
  CHECK_NOTNULL(full_info_cloud);
  CHECK_NOTNULL(range_mat);
  CHECK_NOTNULL(signal_mat);

  AlgorithmSettings settings;
  float verticalAngleSeq, horizonAngleSeq, rangeSeq;
  std::size_t rowIdnSeq, columnIdnSeq, indexSeq;
  for (std::size_t i = start; i < end; ++i) {
    common::Point_t& curPoint = cloud->points[i];
    const float squaredXY = curPoint.x * curPoint.x + curPoint.y * curPoint.y;

    verticalAngleSeq = atan2(curPoint.z, sqrt(squaredXY)) * 180 / M_PI;
    rowIdnSeq = std::trunc(
        (verticalAngleSeq + settings.ang_bottom) / settings.ang_res_y);
    if (rowIdnSeq < 0 || rowIdnSeq >= settings.N_SCAN)
      continue;

    horizonAngleSeq = atan2(curPoint.x, curPoint.y) * 180 / M_PI;

    columnIdnSeq = std::trunc(
        settings.Horizon_SCAN / 2 -
        (horizonAngleSeq - 90.0) / settings.ang_res_x);
    if (columnIdnSeq >= settings.Horizon_SCAN)
      columnIdnSeq -= settings.Horizon_SCAN;

    rangeSeq = sqrt(squaredXY + curPoint.z * curPoint.z);
    range_mat->at<float>(rowIdnSeq, columnIdnSeq) = rangeSeq;
    signal_mat->at<float>(rowIdnSeq, columnIdnSeq) = curPoint.intensity;

    indexSeq = columnIdnSeq + rowIdnSeq * settings.Horizon_SCAN;
    full_info_cloud->points[indexSeq].x = rangeSeq;
    full_info_cloud->points[indexSeq].y = curPoint.intensity;

    // curPoint.intensity = static_cast<float>(rowIdnSeq) +
    // static_cast<float>(columnIdnSeq) / 10000.0;
    full_cloud->points[indexSeq] = curPoint;
  }
}

}  // namespace preproc
