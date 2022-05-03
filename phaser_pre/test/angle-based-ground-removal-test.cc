#include "phaser_pre/algorithm/angle-based-ground-removal.h"

#include <Eigen/Dense>
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
#include "phaser_pre/algorithm/image-projection.h"

namespace preproc {

class AngleBasedGroundRemovalTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    ds_ = std::make_unique<data::DatasourcePly>();
    CHECK_NOTNULL(ds_);
    ds_->setDatasetFolder("./test_clouds/os0/");
  }

  data::DatasourcePlyPtr ds_;
};

TEST_F(AngleBasedGroundRemovalTest, GroundRemovalSeqTest) {
  CHECK(ds_);
  AngleBasedGroundRemoval gnd_removal;
  ImageProjection proj;
  ds_->subscribeToPointClouds([&](const model::PointCloudPtr& cloud) {
    CHECK(cloud);
    const ProjectionResult result = proj.projectPointCloudSequential(cloud);
    const GroundRemovalResult ground_result =
        gnd_removal.removeGroundSeq(result.getFullCloud());

    Eigen::MatrixXf ground;
    cv::cv2eigen(ground_result.getGroundMat(), ground);
    EXPECT_TRUE((ground.array() > 0).any());
  });
  ds_->startStreaming(1);
}

TEST_F(AngleBasedGroundRemovalTest, GroundRemovalVecTest) {
  CHECK(ds_);
  AngleBasedGroundRemoval gnd_removal;
  ImageProjection proj;
  ds_->subscribeToPointClouds([&](const model::PointCloudPtr& cloud) {
    CHECK(cloud);
    const ProjectionResult result = proj.projectPointCloud(cloud);
    const GroundRemovalResult ground_result =
        gnd_removal.removeGround(result.getFullCloud());

    Eigen::MatrixXf ground;
    cv::cv2eigen(ground_result.getGroundMat(), ground);
    EXPECT_TRUE((ground.array() > 0).any());
  });
  ds_->startStreaming(1);
}

TEST_F(AngleBasedGroundRemovalTest, GroundRemovalVecSeqComparisonTest) {
  CHECK(ds_);
  AngleBasedGroundRemoval gnd_removal;
  ImageProjection proj;
  ds_->subscribeToPointClouds([&](const model::PointCloudPtr& cloud) {
    CHECK(cloud);
    const ProjectionResult result = proj.projectPointCloud(cloud);
    const GroundRemovalResult ground_result_vec =
        gnd_removal.removeGround(result.getFullCloud());
    const GroundRemovalResult ground_result_seq =
        gnd_removal.removeGroundSeq(result.getFullCloud());

    Eigen::MatrixXf ground_vec, ground_seq;
    cv::cv2eigen(ground_result_vec.getGroundMat(), ground_vec);
    cv::cv2eigen(ground_result_seq.getGroundMat(), ground_seq);
    EXPECT_TRUE((ground_vec.array() > 0).any());
    EXPECT_TRUE((ground_seq.array() > 0).any());
    EXPECT_NEAR(ground_vec.nonZeros(), ground_seq.nonZeros(), 5);
  });
  ds_->startStreaming(1);
}

}  // namespace preproc

MAPLAB_UNITTEST_ENTRYPOINT
