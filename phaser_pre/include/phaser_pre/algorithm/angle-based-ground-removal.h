#ifndef PHASER_PRE_ALGORITHM_ANGLE_BASED_GROUND_REMOVAL_H_
#define PHASER_PRE_ALGORITHM_ANGLE_BASED_GROUND_REMOVAL_H_

#include <opencv2/core/mat.hpp>

#include "phaser/model/point-cloud.h"
#include "phaser_pre/common/vec-helper.h"
#include "phaser_pre/model/ground-removal-result.h"

namespace preproc {

class AngleBasedGroundRemoval {
 public:
  GroundRemovalResult removeGroundSeq(common::PointCloud_tPtr cloud);
  GroundRemovalResult removeGround(common::PointCloud_tPtr cloud);

 private:
  void removeGroundForIndex(
      common::PointCloud_tPtr cloud, const std::size_t start,
      const std::size_t end, cv::Mat* ground_mat);

  AlgorithmSettings settings_;
};

}  // namespace preproc

#endif  // PHASER_PRE_ALGORITHM_ANGLE_BASED_GROUND_REMOVAL_H_
