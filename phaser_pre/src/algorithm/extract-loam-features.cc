#include "phaser_pre/algorithm/extract-loam-features.h"

#include <glog/logging.h>

namespace preproc {

FeatureExtractionResult ExtractLoamFeatures::extractFeatures(
    const SegmentationResult& seg_result) {
  FeatureExtractionResult result;
  common::PointCloud_tPtr corner_points_sharp =
      CHECK_NOTNULL(result.getCornerPointsSharp());
  common::PointCloud_tPtr corner_points_less_sharp =
      CHECK_NOTNULL(result.getCornerPointsLessSharp());
  common::PointCloud_tPtr surf_points_flat =
      CHECK_NOTNULL(result.getSurfPointsFlat());
  common::PointCloud_tPtr surf_points_less_flat =
      CHECK_NOTNULL(result.getSurfPointsLessFlat());

  return result;
}

}  // namespace preproc
