#include "phaser_pre/algorithm/cloud-segmentation.h"

#include <glog/logging.h>

namespace preproc {

SegmentationResult CloudSegmentation::segment(
    const ProjectionResult& proj_result, const ClusterResult& cluster_result,
    const GroundRemovalResult& ground_result) {
  SegmentationResult seg_result(settings_);
  const cv::Mat& ground_mat = ground_result.getGroundMat();
  const cv::Mat& range_mat = proj_result.getRangeMat();
  const cv::Mat& label_mat = cluster_result.getLabelMat();
  const cv::Mat& occ_mat = cluster_result.getOccMat();
  std::vector<int>& start_index = seg_result.getStartRingIndex();
  std::vector<int>& end_index = seg_result.getEndRingIndex();
  common::PointCloud_tPtr& seg_cloud = seg_result.getSegmentedCloud();
  common::PointCloud_tPtr& seg_info_cloud = seg_result.getSegmentedInfoCloud();
  common::PointCloud_tPtr full_cloud = proj_result.getFullCloud();
  common::PointCloud_tPtr full_info_cloud = proj_result.getFullInfoCloud();
  CHECK_NOTNULL(seg_cloud);
  CHECK_NOTNULL(seg_info_cloud);
  CHECK_NOTNULL(full_cloud);
  CHECK_NOTNULL(full_info_cloud);

  int size_of_cloud = 0u;
  for (uint32_t i = 0u; i < settings_.N_SCAN; ++i) {
    start_index[i] = size_of_cloud - 1 + 5;
    for (uint32_t j = 0u; j < settings_.Horizon_SCAN; ++j) {
      const int8_t ground = ground_mat.at<int8_t>(i, j);
      const float range = range_mat.at<float>(i, j);
      const int label = label_mat.at<int>(i, j);
      const int occ = occ_mat.at<int>(0, label);
      if (ground == 1 || range > 50 || label <= 0 || occ < 9) {
        continue;
      }
      seg_result.getGroundFlag()[size_of_cloud] = false;
      seg_result.getColumnIndex()[size_of_cloud] = j;
      seg_result.getRange()[size_of_cloud] = range;
      seg_cloud->push_back(full_cloud->points[j + i * settings_.Horizon_SCAN]);
      seg_info_cloud->push_back(
          full_info_cloud->points[j + i * settings_.Horizon_SCAN]);
      ++size_of_cloud;
    }
    end_index[i] = size_of_cloud - 1 - 5;
  }

  return seg_result;
}

}  // namespace preproc
