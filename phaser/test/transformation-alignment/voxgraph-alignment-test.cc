#include "packlo/backend/registration/sph-registration.h"
#include "packlo/common/data/datasource-ply.h"
#include "packlo/common/metric-utils.h"
#include "packlo/common/test/testing-entrypoint.h"
#include "packlo/common/test/testing-predicates.h"

#include <gtest/gtest.h>
#include <memory>

namespace transformation {

class VoxgraphAlignmentTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    ds_ = std::make_unique<data::DatasourcePly>();
    ds_->setDatasetFolder(
        "/home/berlukas/Documents/"
        "workspace/maplab/src/packlo/test/test-data/gonzen/"
        "easy-submaps/");
  }

  data::DatasourcePlyPtr ds_;
};

TEST_F(VoxgraphAlignmentTest, TransformVoxgraphEasy) {
  CHECK(ds_);
  registration::BaseRegistrationPtr reg =
      std::make_unique<registration::SphRegistration>();
  model::RegistrationResult result;
  model::PointCloudPtr prev_cloud = nullptr;
  ds_->subscribeToPointClouds([&] (const model::PointCloudPtr& cloud) {
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
    EXPECT_TRUE(result.foundSolutionForRotation());
    EXPECT_TRUE(result.foundSolutionForTranslation());

    // Check that the Hausdorff distance decreased after the registration.
    ASSERT_LE(
        common::MetricUtils::HausdorffDistance(
            prev_cloud, result.getRegisteredCloud()),
        initHausdorff);
  });
  ds_->startStreaming();
}

}  // namespace transformation

MAPLAB_UNITTEST_ENTRYPOINT
