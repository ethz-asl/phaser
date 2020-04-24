#include "phaser_pre/algorithm/calc-smoothness.h"

namespace preproc {

SmoothnessResult CalcSmoothness::compute(const SegmentationResult& seg_result) {
  SmoothnessResult result;
  std::vector<smoothness_t>& smoothness = result.getSmoothness();
  const common::PointCloud_tPtr& seg_cloud = seg_result.getSegmentedCloud();
  const std::size_t n_points = seg_cloud->size() - 5;
  const std::vector<float>& range = seg_result.getRange();
  for (std::size_t i = 5u; i < n_points; ++i) {
    const float diff_range = range[i - 5] + range[i - 4] + range[i - 3] +
                             range[i - 2] + range[i - 1] - range[i] * 10 +
                             range[i + 1] + range[i + 2] + range[i + 3] +
                             range[i + 4] + range[i + 5];
    smoothness[i].first = diff_range * diff_range;
    smoothness[i].second = i;
  }

  return result;
}

}  // namespace preproc
