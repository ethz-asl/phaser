#ifndef PHASER_PRE_MODEL_GROUND_REMOVAL_RESULT_H_
#define PHASER_PRE_MODEL_GROUND_REMOVAL_RESULT_H_

#include <opencv2/core/mat.hpp>

namespace preproc {

class GroundRemovalResult {
 public:
  explicit GroundRemovalResult(cv::Mat&& ground_mat);

  const cv::Mat& getGroundMat() const;

 private:
  cv::Mat ground_mat_;
};

}  // namespace preproc

#endif  // PHASER_PRE_MODEL_GROUND_REMOVAL_RESULT_H_
