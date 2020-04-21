#ifndef PHASER_PRE_ALGORITHM_IMAGE_PROJECTION_H_
#define PHASER_PRE_ALGORITHM_IMAGE_PROJECTION_H_

#include "phaser_pre/model/projection-result.h"

namespace preproc {

class ImageProjection {
 public:
  ImageProjection();

  ProjectionResult projectPointCloudSequential(model::PointCloudPtr cloud);
  ProjectionResult projectPointCloud(model::PointCloudPtr cloud);

 private:
  void projectPointCloudSequentialImpl(
      common::PointCloud_tPtr cloud, const std::size_t start,
      const std::size_t end, common::PointCloud_tPtr full_cloud,
      common::PointCloud_tPtr full_info_cloud, cv::Mat* range_mat,
      cv::Mat* signal_mat);
};

}  // namespace preproc

#endif  // PHASER_PRE_ALGORITHM_IMAGE_PROJECTION_H_
