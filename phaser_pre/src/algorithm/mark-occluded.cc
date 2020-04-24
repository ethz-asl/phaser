#include "phaser_pre/algorithm/mark-occluded.h"

namespace preproc {

OcclusionResult MarkOccluded::compute(const SegmentationResult& seg_result) {
  OcclusionResult result;
  const common::PointCloud_tPtr& seg_cloud = seg_result.getSegmentedCloud();
  const std::size_t n_points = seg_cloud->size() - 6;

  const std::vector<float>& range = seg_result.getRange();
  const std::vector<uint32_t>& col_ind = seg_result.getColumnIndex();
  std::vector<int>& neighbor_picked = result.getPickedNeighbors();
  for (int i = 5; i < n_points; ++i) {
    const float depth1 = range[i];
    const float depth2 = range[i + 1];
    const int column_diff =
        std::abs(static_cast<int>(col_ind[i + 1] - col_ind[i]));

    if (column_diff < 10) {
      if (depth1 - depth2 > 0.3) {
        neighbor_picked[i - 5] = 1;
        neighbor_picked[i - 4] = 1;
        neighbor_picked[i - 3] = 1;
        neighbor_picked[i - 2] = 1;
        neighbor_picked[i - 1] = 1;
        neighbor_picked[i] = 1;
      } else if (depth2 - depth1 > 0.3) {
        neighbor_picked[i + 1] = 1;
        neighbor_picked[i + 2] = 1;
        neighbor_picked[i + 3] = 1;
        neighbor_picked[i + 4] = 1;
        neighbor_picked[i + 5] = 1;
        neighbor_picked[i + 6] = 1;
      }
    }

    const float diff1 = std::abs(range[i - 1] - range[i]);
    const float diff2 = std::abs(range[i + 1] - range[i]);

    if (diff1 > 0.02 * range[i] && diff2 > 0.02 * range[i]) {
      neighbor_picked[i] = 1;
    }
  }

  return result;
}

}  // namespace preproc
