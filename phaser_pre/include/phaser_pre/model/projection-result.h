#ifndef PHASER_PRE_MODEL_PROJECTION_RESULT_H_
#define PHASER_PRE_MODEL_PROJECTION_RESULT_H_

#include <opencv2/core/mat.hpp>

#include "phaser/model/point-cloud.h"

namespace preproc {

class ProjectionResult {
 public:
  ProjectionResult(
      common::PointCloud_tPtr full_cloud,
      common::PointCloud_tPtr full_info_cloud, const cv::Mat& range_mat,
      const cv::Mat& signal_mat);

  common::PointCloud_tPtr getFullCloud() const;
  common::PointCloud_tPtr getFullInfoCloud() const;
  cv::Mat getRangeMat() const;
  cv::Mat getSignalMat() const;

 private:
  common::PointCloud_tPtr full_cloud_;
  common::PointCloud_tPtr full_info_cloud_;
  cv::Mat range_mat_;
  cv::Mat signal_mat_;
};

}  // namespace preproc

#endif  // PHASER_PRE_MODEL_PROJECTION_RESULT_H_
