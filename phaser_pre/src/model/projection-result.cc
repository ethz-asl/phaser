#include "phaser_pre/model/projection-result.h"

#include <glog/logging.h>

namespace preproc {

ProjectionResult::ProjectionResult(
    common::PointCloud_tPtr full_cloud, common::PointCloud_tPtr full_info_cloud,
    const cv::Mat& range_mat, const cv::Mat& signal_mat)
    : range_mat_(range_mat),
      signal_mat_(signal_mat),
      full_cloud_(CHECK_NOTNULL(full_cloud)),
      full_info_cloud_(CHECK_NOTNULL(full_info_cloud)) {}

common::PointCloud_tPtr ProjectionResult::getFullCloud() const {
  return full_cloud_;
}

common::PointCloud_tPtr ProjectionResult::getFullInfoCloud() const {
  return full_info_cloud_;
}

cv::Mat ProjectionResult::getRangeMat() const {
  return range_mat_;
}

cv::Mat ProjectionResult::getSignalMat() const {
  return signal_mat_;
}

}  // namespace preproc
