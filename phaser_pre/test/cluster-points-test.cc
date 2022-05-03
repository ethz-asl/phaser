#include "phaser_pre/algorithm/cluster-points.h"

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

class ClusterPointsTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    ds_ = std::make_unique<data::DatasourcePly>();
    CHECK_NOTNULL(ds_);
    ds_->setDatasetFolder("./test_clouds/os0/");
  }

  data::DatasourcePlyPtr ds_;
};

TEST_F(ClusterPointsTest, FindLabelsAndClusterTest) {
  CHECK(ds_);
  ImageProjection proj;
  ClusterPoints cluster;
  ds_->subscribeToPointClouds([&](const model::PointCloudPtr& cloud) {
    CHECK(cloud);
    const ProjectionResult proj_result = proj.projectPointCloud(cloud);
    const ClusterResult cluster_result =
        cluster.cluster(proj_result.getRangeMat(), proj_result.getSignalMat());

    Eigen::MatrixXf occ, labels;
    cv::cv2eigen(cluster_result.getOccMat(), occ);
    cv::cv2eigen(cluster_result.getLabelMat(), labels);
    EXPECT_TRUE((occ.array() > 0).any());
    EXPECT_TRUE((labels.array() > 0).any());
  });
  ds_->startStreaming(1);
}

}  // namespace preproc

MAPLAB_UNITTEST_ENTRYPOINT
