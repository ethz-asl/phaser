#include "phaser_pre/algorithm/image-gradient.h"

#include <opencv2/highgui/highgui.hpp>

#include "opencv2/imgproc.hpp"

namespace preproc {

GradientResult ImageGradient::compute(const cv::Mat& range_mat) {
  GradientResult result(settings_);
  cv::Mat& range_grad = result.getRangeGradient();
  constexpr int kernel_size = 3;
  constexpr int scale = 1;
  constexpr int delta = 1;
  constexpr int ddepth = CV_32F;
  /*
  cv::Laplacian(
      range_mat, range_grad, ddepth, kernel_size, scale, delta,
      cv::BORDER_DEFAULT);
      */
  cv::Sobel(
      range_mat, range_grad, ddepth, 1, 0, kernel_size, scale, delta,
      cv::BORDER_DEFAULT);

  return result;
}

}  // namespace preproc
