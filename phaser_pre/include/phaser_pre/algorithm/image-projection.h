#ifndef PHASER_PRE_ALGORITHM_IMAGE_PROJECTION_H_
#define PHASER_PRE_ALGORITHM_IMAGE_PROJECTION_H_

#include <opencv2/core/mat.hpp>

#include "phaser/model/point-cloud.h"

namespace preproc {

class ImageProjection {
 public:
  ImageProjection();

  void projectPointCloud(model::PointCloudPtr cloud);

 private:
  common::PointCloud_tPtr full_cloud_;
  common::PointCloud_tPtr full_info_cloud_;
  cv::Mat range_mat_;
  cv::Mat signal_mat_;
};

}  // namespace preproc

#endif  // PHASER_PRE_ALGORITHM_IMAGE_PROJECTION_H_
