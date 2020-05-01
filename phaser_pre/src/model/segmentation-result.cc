#include "phaser_pre/model/segmentation-result.h"

namespace preproc {

SegmentationResult::SegmentationResult(const AlgorithmSettings& settings) {
  start_ring_index_.assign(settings.N_SCAN, 0);
  end_ring_index_.assign(settings.N_SCAN, 0);

  const uint32_t n_points = settings.N_SCAN * settings.Horizon_SCAN;
  ground_flag_.assign(n_points, false);
  col_ind_.assign(n_points, 0);
  range_.assign(n_points, 0);
  segmented_cloud_.reset(new common::PointCloud_t);
  segmented_info_cloud_.reset(new common::PointCloud_t);
}

const std::vector<int>& SegmentationResult::getStartRingIndex() const {
  return start_ring_index_;
}

std::vector<int>& SegmentationResult::getStartRingIndex() {
  return start_ring_index_;
}

const std::vector<int>& SegmentationResult::getEndRingIndex() const {
  return end_ring_index_;
}

std::vector<int>& SegmentationResult::getEndRingIndex() {
  return end_ring_index_;
}

float SegmentationResult::getStartOrientation() const {
  return start_orientation_;
}

float& SegmentationResult::getStartOrientation() {
  return start_orientation_;
}

float SegmentationResult::getEndOrientation() const {
  return end_orientation_;
}

float& SegmentationResult::getEndOrientation() {
  return end_orientation_;
}

float SegmentationResult::getOrientationDiff() const {
  return orientation_diff_;
}

float& SegmentationResult::getOrientationDiff() {
  return orientation_diff_;
}

const std::vector<bool>& SegmentationResult::getGroundFlag() const {
  return ground_flag_;
}

std::vector<bool>& SegmentationResult::getGroundFlag() {
  return ground_flag_;
}

const std::vector<uint32_t>& SegmentationResult::getColumnIndex() const {
  return col_ind_;
}

std::vector<uint32_t>& SegmentationResult::getColumnIndex() {
  return col_ind_;
}

const std::vector<float>& SegmentationResult::getRange() const {
  return range_;
}

std::vector<float>& SegmentationResult::getRange() {
  return range_;
}

common::PointCloud_tPtr& SegmentationResult::getSegmentedCloud() {
  return segmented_cloud_;
}

const common::PointCloud_tPtr& SegmentationResult::getSegmentedCloud() const {
  return segmented_cloud_;
}

common::PointCloud_tPtr& SegmentationResult::getSegmentedInfoCloud() {
  return segmented_info_cloud_;
}

const common::PointCloud_tPtr& SegmentationResult::getSegmentedInfoCloud()
    const {
  return segmented_info_cloud_;
}

}  // namespace preproc
