#include "phaser/backend/registration/sph-registration.h"
#include "phaser/common/data/datasource-ply.h"
#include "phaser/common/metric-utils.h"
#include "phaser/common/test/testing-entrypoint.h"
#include "phaser/common/test/testing-predicates.h"

#include <gtest/gtest.h>
#include <memory>

namespace transformation {

class VoxgraphAlignmentTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    ds_ = std::make_unique<data::DatasourcePly>();
    ds_->setDatasetFolder("./test_clouds/gonzen/easy-submaps/");
  }

  data::DatasourcePlyPtr ds_;
};

TEST_F(VoxgraphAlignmentTest, TransformVoxgraphEasy) {
  CHECK(ds_);
  registration::BaseRegistrationPtr reg =
      std::make_unique<registration::SphRegistration>();
  model::RegistrationResult result;
  model::PointCloudPtr prev_cloud = nullptr;
  ds_->subscribeToPointClouds([&](const model::PointCloudPtr& cloud) {
    CHECK(cloud);
    if (prev_cloud == nullptr) {
      prev_cloud = cloud;
      prev_cloud->initialize_kd_tree();
      return;
    }

    // Register the point clouds.
    const float initHausdorff =
        common::MetricUtils::HausdorffDistance(prev_cloud, cloud);
    cloud->initialize_kd_tree();
    result = reg->registerPointCloud(prev_cloud, cloud);
    // EXPECT_TRUE(result.foundSolutionForRotation());
    // EXPECT_TRUE(result.foundSolutionForTranslation());

    // Check that the Hausdorff distance decreased after the registration.
    ASSERT_LE(
        common::MetricUtils::HausdorffDistance(
            prev_cloud, result.getRegisteredCloud()),
        50);
  });
  ds_->startStreaming(1);
}

}  // namespace transformation

MAPLAB_UNITTEST_ENTRYPOINT
