#ifndef PHASER_PRE_MODEL_CLUSTER_RESULT_H_
#define PHASER_PRE_MODEL_CLUSTER_RESULT_H_

#include <opencv2/core/mat.hpp>

namespace preproc {

class ClusterResult {
 public:
  ClusterResult(cv::Mat&& occ_mat, cv::Mat&& label_mat);

  const cv::Mat& getOccMat() const;
  const cv::Mat& getLabelMat() const;

 private:
  cv::Mat occ_mat_;
  cv::Mat label_mat_;
};

}  // namespace preproc

#endif  // PHASER_PRE_MODEL_CLUSTER_RESULT_H_
