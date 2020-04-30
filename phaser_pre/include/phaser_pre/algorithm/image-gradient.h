#ifndef PHASER_PRE_ALGORITHM_IMAGE_GRADIENT_H_
#define PHASER_PRE_ALGORITHM_IMAGE_GRADIENT_H_

#include "phaser_pre/common/vec-helper.h"
#include "phaser_pre/model/gradient-result.h"

namespace preproc {

class ImageGradient {
 public:
  GradientResult compute(const cv::Mat& range_mat);

 private:
  AlgorithmSettings settings_;
};

}  // namespace preproc

#endif  // PHASER_PRE_ALGORITHM_IMAGE_GRADIENT_H_
