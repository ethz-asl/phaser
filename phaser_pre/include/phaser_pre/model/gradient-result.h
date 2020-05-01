#ifndef PHASER_PRE_MODEL_GRADIENT_RESULT_H_
#define PHASER_PRE_MODEL_GRADIENT_RESULT_H_

#include <opencv2/core/mat.hpp>

#include "phaser_pre/common/vec-helper.h"

namespace preproc {

class GradientResult {
 public:
  explicit GradientResult(const AlgorithmSettings& settings);
  explicit GradientResult(const cv::Mat& range_gradient);

  cv::Mat& getRangeGradient();
  const cv::Mat& getRangeGradient() const;

 private:
  cv::Mat range_gradient_;
};

}  // namespace preproc

#endif  //  PHASER_PRE_MODEL_GRADIENT_RESULT_H_
