#include "phaser_pre/commands/segment-cloud-cmd.h"

namespace preproc {

void SegmentCloudCmd::execute(model::PointCloudPtr cloud) {
  const ProjectionResult proj_result = proj_.projectPointCloud(cloud);
  const ClusterResult cluster_result =
      cluster_.cluster(proj_result.getRangeMat(), proj_result.getSignalMat());
  const GroundRemovalResult ground_result =
      gnd_removal_.removeGround(proj_result.getFullCloud());
  const SegmentationResult seg_result =
      seg_.segment(proj_result, cluster_result, ground_result);

  const common::PointCloud_tPtr& seg_cloud = seg_result.getSegmentedCloud();
  cloud->getRawCloud() = seg_cloud;
}

}  // namespace preproc
