#include "phaser_pre/model/gradient-result.h"

namespace preproc {

GradientResult::GradientResult(const AlgorithmSettings& settings) {
  range_gradient_ = cv::Mat(
      settings.N_SCAN, settings.Horizon_SCAN, CV_32F, cv::Scalar::all(0));
}

GradientResult::GradientResult(const cv::Mat& range_gradient)
    : range_gradient_(range_gradient) {}

cv::Mat& GradientResult::getRangeGradient() {
  return range_gradient_;
}

const cv::Mat& GradientResult::getRangeGradient() const {
  return range_gradient_;
}

}  // namespace preproc
