#include "phaser_pre/model/segmentation-result.h"

namespace preproc {

const std::vector<int>& CloudSegmentation::getStartRingIndex() const {
  return start_ring_index_;
}

std::vector<int>& CloudSegmentation::getStartRingIndex() {
  return start_ring_index_;
}

const std::vector<int>& CloudSegmentation::getEndRingIndex() const {
  return end_ring_index_;
}

std::vector<int>& CloudSegmentation::getEndRingIndex() {
  return end_ring_index_;
}

float CloudSegmentation::getStartOrientation() const {
  return start_orientation_;
}

float& CloudSegmentation::getStartOrientation() {
  return start_orientation_;
}

float CloudSegmentation::getEndOrientation() const {
  return end_orientation_;
}

float& CloudSegmentation::getEndOrientation() {
  return end_orientation_;
}

float CloudSegmentation::getOrientationDiff() const {
  return orientation_diff_;
}

float& CloudSegmentation::getOrientationDiff() {
  return orientation_diff_;
}

const std::vector<bool>& CloudSegmentation::getGroundFlag() const {
  return ground_flag_;
}

std::vector<bool>& CloudSegmentation::getGroundFlag() {
  return ground_flag_;
}

const std::vector<uint32_t>& CloudSegmentation::getColumnIndex() const {
  return col_ind_;
}

std::vector<uint32_t>& CloudSegmentation::getColumnIndex() {
  return col_ind_;
}

const std::vector<float>& CloudSegmentation::getRange() const {
  return range_;
}

std::vector<float>& CloudSegmentation::getRange() {
  return range_;
}

}  // namespace preproc
