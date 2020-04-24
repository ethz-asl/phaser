#include "phaser_pre/algorithm/extract-loam-features.h"

#include <glog/logging.h>

namespace preproc {

FeatureExtractionResult ExtractLoamFeatures::extractFeatures(
    const SegmentationResult& seg_result, const SmoothnessResult& smooth_result,
    const OcclusionResult& occ_result) {
  FeatureExtractionResult result;
  common::PointCloud_tPtr corner_points_sharp =
      CHECK_NOTNULL(result.getCornerPointsSharp());
  common::PointCloud_tPtr corner_points_less_sharp =
      CHECK_NOTNULL(result.getCornerPointsLessSharp());
  common::PointCloud_tPtr surf_points_flat =
      CHECK_NOTNULL(result.getSurfPointsFlat());
  common::PointCloud_tPtr surf_points_less_flat =
      CHECK_NOTNULL(result.getSurfPointsLessFlat());

  const std::vector<int>& start_index = seg_result.getStartRingIndex();
  const std::vector<int>& end_index = seg_result.getEndRingIndex();
  const std::vector<bool>& ground_flag = seg_result.getGroundFlag();
  const std::vector<uint32_t>& col_ind = seg_result.getColumnIndex();
  common::PointCloud_tPtr seg_cloud =
      CHECK_NOTNULL(seg_result.getSegmentedCloud());

  // intentional copy as we modify the two vectors.
  std::vector<int> picked_neighbors = occ_result.getPickedNeighbors();
  std::vector<smoothness_t> smoothness = smooth_result.getSmoothness();

  // TODO(lbern): expose as flags
  const float edge_threshold = 0.1;
  const float surf_threshold = 0.1;

  std::vector<uint8_t> unlabeled;
  unlabeled.assign(settings_.N_SCAN * settings_.Horizon_SCAN, 0);
  for (uint32_t i = 0u; i < settings_.N_SCAN; ++i) {
    for (uint8_t j = 0; j < 6; ++j) {
      const int sp = (start_index[i] * (6 - j) + end_index[i] * j) / 6;
      const int ep =
          (start_index[i] * (5 - j) + end_index[i] * (j + 1)) / 6 - 1;
      if (sp >= ep) {
        continue;
      }

      std::sort(
          smoothness.begin() + sp, smoothness.begin() + ep,
          [](const smoothness_t& lhs, const smoothness_t& rhs) {
            return lhs.first < rhs.first;
          });

      int largest_picked_num = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = smoothness[k].second;
        if (picked_neighbors[ind] == 0 &&
            smoothness[ind].first > edge_threshold &&
            ground_flag[ind] == false) {
          largest_picked_num++;
          if (largest_picked_num <= 2) {
            unlabeled[ind] = 2;
            corner_points_sharp->push_back(seg_cloud->points[ind]);
            corner_points_less_sharp->push_back(seg_cloud->points[ind]);
          } else if (largest_picked_num <= 20) {
            unlabeled[ind] = 1;
            corner_points_less_sharp->push_back(seg_cloud->points[ind]);
          } else {
            break;
          }

          picked_neighbors[ind] = 1;
          for (int l = 1; l <= 5; ++l) {
            int columnDiff = std::abs(
                static_cast<int>(col_ind[ind + l] - col_ind[ind + l - 1]));
            if (columnDiff > 10)
              break;
            picked_neighbors[ind + l] = 1;
          }
          for (int l = -1; l >= -5; --l) {
            int columnDiff = std::abs(
                static_cast<int>(col_ind[ind + l] - col_ind[ind + l + 1]));
            if (columnDiff > 10)
              break;
            picked_neighbors[ind + l] = 1;
          }
        }
      }

      int smalled_picked_num = 0;
      for (int k = sp; k <= ep; k++) {
        int ind = smoothness[k].second;
        if (picked_neighbors[ind] == 0 &&
            smoothness[ind].first < surf_threshold &&
            ground_flag[ind] == false) {
          surf_points_flat->push_back(seg_cloud->points[ind]);
          unlabeled[ind] = 3;

          if (++smalled_picked_num >= 4) {
            break;
          }

          picked_neighbors[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            int columnDiff = std::abs(
                static_cast<int>(col_ind[ind + l] - col_ind[ind + l - 1]));
            if (columnDiff > 10)
              break;

            picked_neighbors[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            int columnDiff = std::abs(
                static_cast<int>(col_ind[ind + l] - col_ind[ind + l + 1]));
            if (columnDiff > 10)
              break;

            picked_neighbors[ind + l] = 1;
          }
        }
      }
      for (int k = sp; k <= ep; ++k) {
        if (unlabeled[k] <= 0) {
          surf_points_less_flat->push_back(seg_cloud->points[k]);
        }
      }
    }
  }

  return result;
}

}  // namespace preproc
