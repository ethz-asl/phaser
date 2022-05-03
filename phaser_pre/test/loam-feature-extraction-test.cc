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
#include "phaser_pre/algorithm/calc-smoothness.h"
#include "phaser_pre/algorithm/cloud-segmentation.h"
#include "phaser_pre/algorithm/cluster-points.h"
#include "phaser_pre/algorithm/extract-loam-features.h"
#include "phaser_pre/algorithm/image-projection.h"
#include "phaser_pre/algorithm/mark-occluded.h"

namespace preproc {

class LoamFeatureExtractionTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    ds_ = std::make_unique<data::DatasourcePly>();
    CHECK_NOTNULL(ds_);
    ds_->setDatasetFolder("./test_clouds/os0/");
  }

  data::DatasourcePlyPtr ds_;
};

TEST_F(LoamFeatureExtractionTest, FeatureExtractionTest) {
  CHECK(ds_);
  ImageProjection proj;
  ClusterPoints cluster;
  AngleBasedGroundRemoval gnd_removal;
  CloudSegmentation seg;
  MarkOccluded marker;
  ExtractLoamFeatures features;
  CalcSmoothness smoothness;
  const uint32_t skip_n = 1u;
  uint32_t cloud_counter = 0u;
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
    const SmoothnessResult smooth_result = smoothness.compute(seg_result);
    FeatureExtractionResult ext_result =
        features.extractFeatures(seg_result, smooth_result, occ_result);

    common::PointCloud_tPtr corner_sharp = ext_result.getCornerPointsSharp();
    common::PointCloud_tPtr corner_less_sharp =
        ext_result.getCornerPointsLessSharp();
    common::PointCloud_tPtr surf_flat = ext_result.getSurfPointsFlat();
    common::PointCloud_tPtr surf_less_flat = ext_result.getSurfPointsLessFlat();

    EXPECT_NE(nullptr, corner_sharp);
    EXPECT_NE(nullptr, corner_less_sharp);
    EXPECT_NE(nullptr, surf_flat);
    EXPECT_NE(nullptr, surf_less_flat);

    EXPECT_GE(corner_sharp->size(), 0);
    EXPECT_GE(corner_less_sharp->size(), 0);
    EXPECT_GE(surf_flat->size(), 0);
    EXPECT_GE(surf_less_flat->size(), 0);
  });
  ds_->startStreaming(2);
}

}  // namespace preproc

MAPLAB_UNITTEST_ENTRYPOINT
