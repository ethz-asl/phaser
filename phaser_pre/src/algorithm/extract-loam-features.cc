#include "phaser_pre/algorithm/extract-loam-features.h"

#include <glog/logging.h>

namespace preproc {

FeatureExtractionResult ExtractLoamFeatures::extractFeatures(
    const SegmentationResult& seg_result,
    const SmoothnessResult& smooth_result) {
  FeatureExtractionResult result;
  common::PointCloud_tPtr corner_points_sharp =
      CHECK_NOTNULL(result.getCornerPointsSharp());
  common::PointCloud_tPtr corner_points_less_sharp =
      CHECK_NOTNULL(result.getCornerPointsLessSharp());
  common::PointCloud_tPtr surf_points_flat =
      CHECK_NOTNULL(result.getSurfPointsFlat());
  common::PointCloud_tPtr surf_points_less_flat =
      CHECK_NOTNULL(result.getSurfPointsLessFlat());

  const std::vector<int>& start_index = seg_result.getStartRingIndex();
  const std::vector<int>& end_index = seg_result.getEndRingIndex();
  std::vector<smoothness_t> smoothness = smooth_result.getSmoothness();

  for (uint32_t i = 0u; i < settings_.N_SCAN; ++i) {
    for (uint8_t j = 0; j < 6; ++j) {
      const int sp = (start_index[i] * (6 - j) + end_index[i] * j) / 6;
      const int ep =
          (start_index[i] * (5 - j) + end_index[i] * (j + 1)) / 6 - 1;
      if (sp >= ep) {
        continue;
      }

      std::sort(
          smoothness.begin() + sp, smoothness.begin() + ep,
          [](const smoothness_t& lhs, const smoothness_t& rhs) {
            return lhs.first < rhs.first;
          });

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = smoothness[k].second;
        /*
        if (cloudNeighborPicked[ind] == 0
            && cloudCurvature[ind] > edgeThreshold
            && segInfo.segmentedCloudGroundFlag[ind] == false) {
              */
      }
    }
  }

  return result;
}

}  // namespace preproc
