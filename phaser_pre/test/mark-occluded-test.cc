#include "phaser_pre/algorithm/mark-occluded.h"

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>
#include <memory>
#include <opencv2/core/eigen.hpp>
#include <random>

#include "phaser/common/data/datasource-ply.h"
#include "phaser/common/test/testing-entrypoint.h"
#include "phaser/common/test/testing-predicates.h"
#include "phaser/model/point-cloud.h"
#include "phaser_pre/algorithm/angle-based-ground-removal.h"
#include "phaser_pre/algorithm/cloud-segmentation.h"
#include "phaser_pre/algorithm/cluster-points.h"
#include "phaser_pre/algorithm/image-projection.h"

namespace preproc {

class MarkOccludedTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    ds_ = std::make_unique<data::DatasourcePly>();
    CHECK_NOTNULL(ds_);
    ds_->setDatasetFolder("./test_clouds/os0/");
  }

  data::DatasourcePlyPtr ds_;
};

TEST_F(MarkOccludedTest, FindOccludedTest) {
  CHECK(ds_);
  ImageProjection proj;
  ClusterPoints cluster;
  AngleBasedGroundRemoval gnd_removal;
  CloudSegmentation seg;
  MarkOccluded marker;
  ds_->subscribeToPointClouds([&](const model::PointCloudPtr& cloud) {
    CHECK(cloud);
    const ProjectionResult proj_result = proj.projectPointCloud(cloud);
    const ClusterResult cluster_result =
        cluster.cluster(proj_result.getRangeMat(), proj_result.getSignalMat());
    const GroundRemovalResult ground_result =
        gnd_removal.removeGround(proj_result.getFullCloud());
    const SegmentationResult seg_result =
        seg.segment(proj_result, cluster_result, ground_result);
    const OcclusionResult occ_result = marker.compute(seg_result);

    const std::vector<int>& picked_neighbors = occ_result.getPickedNeighbors();
    const int num_picked =
        std::count(picked_neighbors.begin(), picked_neighbors.end(), 1);
    EXPECT_GT(num_picked, 0);
  });
  ds_->startStreaming(1);
}

}  // namespace preproc

MAPLAB_UNITTEST_ENTRYPOINT
