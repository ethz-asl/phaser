#include <chrono>
#include <cmath>
#include <gtest/gtest.h>
#include <memory>
#include <random>

#include <Eigen/Dense>
#include <eigen-checks/gtest.h>
#include <opencv2/core/eigen.hpp>

#include "phaser/common/data/datasource-ply.h"
#include "phaser/common/test/testing-entrypoint.h"
#include "phaser/common/test/testing-predicates.h"
#include "phaser/model/point-cloud.h"
#include "phaser_pre/algorithm/angle-based-ground-removal.h"
#include "phaser_pre/algorithm/cloud-segmentation.h"
#include "phaser_pre/algorithm/cluster-points.h"
#include "phaser_pre/algorithm/image-projection.h"

namespace preproc {

class CloudSegmentationTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    ds_ = std::make_unique<data::DatasourcePly>();
    CHECK_NOTNULL(ds_);
    ds_->setDatasetFolder("./test_clouds/arche/sigma-level-1/");
  }

  data::DatasourcePlyPtr ds_;
};

TEST_F(CloudSegmentationTest, SegmentCloudTest) {
  CHECK(ds_);
  ImageProjection proj;
  ClusterPoints cluster;
  AngleBasedGroundRemoval gnd_removal;
  CloudSegmentation seg;
  ds_->subscribeToPointClouds([&](const model::PointCloudPtr& cloud) {
    CHECK(cloud);
    const ProjectionResult proj_result = proj.projectPointCloud(cloud);
    const ClusterResult cluster_result =
        cluster.cluster(proj_result.getRangeMat(), proj_result.getSignalMat());
    const GroundRemovalResult ground_result =
        gnd_removal.removeGround(proj_result.getFullCloud());
    const SegmentationResult seg_result =
        seg.segment(proj_result, cluster_result, ground_result);

    const common::PointCloud_tPtr& seg_cloud = seg_result.getSegmentedCloud();
    EXPECT_GT(seg_cloud->size(), 0);
  });
  ds_->startStreaming(1);
}

}  // namespace preproc

MAPLAB_UNITTEST_ENTRYPOINT
