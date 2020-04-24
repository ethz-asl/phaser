#include "phaser_pre/model/feature-extraction-result.h"

namespace preproc {

FeatureExtractionResult::FeatureExtractionResult() {
  corner_points_sharp_.reset(new common::PointCloud_t);
  corner_points_less_sharp_.reset(new common::PointCloud_t);
  surf_points_flat_.reset(new common::PointCloud_t);
  surf_points_less_flat_.reset(new common::PointCloud_t);
}

common::PointCloud_tPtr FeatureExtractionResult::getCornerPointsSharp() {
  return corner_points_sharp_;
}

const common::PointCloud_tPtr& FeatureExtractionResult::getCornerPointsSharp()
    const {
  return corner_points_sharp_;
}

common::PointCloud_tPtr FeatureExtractionResult::getCornerPointsLessSharp() {
  return corner_points_less_sharp_;
}

const common::PointCloud_tPtr&
FeatureExtractionResult::getCornerPointsLessSharp() const {
  return corner_points_less_sharp_;
}

common::PointCloud_tPtr FeatureExtractionResult::getSurfPointsFlat() {
  return surf_points_flat_;
}

const common::PointCloud_tPtr& FeatureExtractionResult::getSurfPointsLessFlat()
    const {
  return surf_points_less_flat_;
}

}  // namespace preproc
