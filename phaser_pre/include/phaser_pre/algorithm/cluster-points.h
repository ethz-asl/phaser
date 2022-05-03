#ifndef PHASER_PRE_ALGORITHM_CLUSTER_POINTS_H_
#define PHASER_PRE_ALGORITHM_CLUSTER_POINTS_H_

#include <opencv2/core/mat.hpp>
#include <vector>

#include "phaser/model/point-cloud.h"
#include "phaser_pre/common/vec-helper.h"
#include "phaser_pre/model/cluster-result.h"

namespace preproc {

class ClusterPoints {
 public:
  ClusterPoints();
  ClusterResult cluster(const cv::Mat& range_mat, const cv::Mat& signal_mat);

 private:
  uint32_t findAndLabelSimilarPoints(
      const cv::Point& p, const int label, const cv::Mat& range_mat,
      const cv::Mat& signal_mat, cv::Mat* label_mat);

  AlgorithmSettings settings_;
  std::vector<cv::Point> neighbors_;
};

}  // namespace preproc

#endif  // PHASER_PRE_ALGORITHM_CLUSTER_POINTS_H_
