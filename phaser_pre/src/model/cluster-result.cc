#include "phaser_pre/model/cluster-result.h"

namespace preproc {

ClusterResult::ClusterResult(cv::Mat&& occ_mat, cv::Mat&& label_mat)
    : occ_mat_(occ_mat), label_mat_(label_mat) {}

const cv::Mat& ClusterResult::getOccMat() const {
  return occ_mat_;
}

const cv::Mat& ClusterResult::getLabelMat() const {
  return label_mat_;
}

}  // namespace preproc
