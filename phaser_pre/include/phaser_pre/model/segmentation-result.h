#ifndef PHASER_PRE_MODEL_SEGMENTATION_RESULT_H_
#define PHASER_PRE_MODEL_SEGMENTATION_RESULT_H_

#include <cstdint>
#include <vector>

#include "phaser/model/point-cloud.h"
#include "phaser_pre/common/vec-helper.h"

namespace preproc {

class SegmentationResult {
 public:
  explicit SegmentationResult(const AlgorithmSettings& settings);

  const std::vector<int>& getStartRingIndex() const;
  std::vector<int>& getStartRingIndex();

  const std::vector<int>& getEndRingIndex() const;
  std::vector<int>& getEndRingIndex();

  float getStartOrientation() const;
  float& getStartOrientation();

  float getEndOrientation() const;
  float& getEndOrientation();

  float getOrientationDiff() const;
  float& getOrientationDiff();

  const std::vector<bool>& getGroundFlag() const;
  std::vector<bool>& getGroundFlag();

  const std::vector<uint32_t>& getColumnIndex() const;
  std::vector<uint32_t>& getColumnIndex();

  const std::vector<float>& getRange() const;
  std::vector<float>& getRange();

  common::PointCloud_tPtr& getSegmentedCloud();
  const common::PointCloud_tPtr& getSegmentedCloud() const;

  common::PointCloud_tPtr& getSegmentedInfoCloud();
  const common::PointCloud_tPtr& getSegmentedInfoCloud() const;

 private:
  std::vector<int> start_ring_index_;
  std::vector<int> end_ring_index_;
  float start_orientation_;
  float end_orientation_;
  float orientation_diff_;
  std::vector<bool> ground_flag_;
  std::vector<uint32_t> col_ind_;
  std::vector<float> range_;
  common::PointCloud_tPtr segmented_cloud_;
  common::PointCloud_tPtr segmented_info_cloud_;
};

}  // namespace preproc

#endif  // PHASER_PRE_MODEL_SEGMENTATION_RESULT_H_
