#ifndef PHASER_PRE_MODEL_FEATURE_EXTRACTION_RESULT_H_
#define PHASER_PRE_MODEL_FEATURE_EXTRACTION_RESULT_H_

#include "phaser/model/point-cloud.h"

namespace preproc {

class FeatureExtractionResult {
 public:
  FeatureExtractionResult();

  common::PointCloud_tPtr getCornerPointsSharp();
  const common::PointCloud_tPtr& getCornerPointsSharp() const;

  common::PointCloud_tPtr getCornerPointsLessSharp();
  const common::PointCloud_tPtr& getCornerPointsLessSharp() const;

  common::PointCloud_tPtr getSurfPointsFlat();
  const common::PointCloud_tPtr& getSurfPointsLessFlat() const;

 private:
  common::PointCloud_tPtr corner_points_sharp_;
  common::PointCloud_tPtr corner_points_less_sharp_;
  common::PointCloud_tPtr surf_points_flat_;
  common::PointCloud_tPtr surf_points_less_flat_;
};

}  // namespace preproc

#endif  // PHASER_PRE_MODEL_FEATURE_EXTRACTION_RESULT_H_
