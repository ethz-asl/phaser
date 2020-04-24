#include "phaser_pre/model/ground-removal-result.h"

namespace preproc {

GroundRemovalResult::GroundRemovalResult(cv::Mat&& ground_mat)
    : ground_mat_(ground_mat) {}

const cv::Mat& GroundRemovalResult::getGroundMat() const {
  return ground_mat_;
}

}  // namespace preproc
