#include "phaser_pre/algorithm/image-projection.h"

#include <chrono>

#define USE_SSE2
#include "phaser_pre/common/sse-mathfun-extension.h"
#include "phaser_pre/common/vec-helper.h"

namespace preproc {

ImageProjection::ImageProjection() {
  rangeMat_ = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
  signalMat_ = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(0));
}

void ImageProjection::projectPointCloud(model::PointCloudPtr cloud) {}

}  // namespace preproc
