#include "packlo/backend/registration/sph-registration.h"
#include "packlo/common/data/datasource-ply.h"
#include "packlo/common/metric-utils.h"
#include "packlo/common/test/testing-entrypoint.h"
#include "packlo/common/test/testing-predicates.h"
#include "packlo/distribution/gaussian.h"

#include <gtest/gtest.h>

namespace uncertainty {

class TransGaussTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    ds_ = std::make_unique<data::DatasourcePly>();
    CHECK_NOTNULL(ds_);
    ds_->setDatasetFolder("./test_clouds/arche/");
    registrator_ = std::make_unique<registration::SphRegistration>();
  }

  data::DatasourcePlyPtr ds_;
  registration::BaseRegistrationPtr registrator_;
};

TEST_F(TransGaussTest, LowUncertainty) {
  CHECK(ds_);

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
    cloud->initialize_kd_tree();
    result = registrator_->registerPointCloud(prev_cloud, cloud);
    prev_cloud = cloud;
    EXPECT_TRUE(result.foundSolutionForRotation());
    EXPECT_TRUE(result.foundSolutionForTranslation());

    common::GaussianPtr uncertainty =
        std::dynamic_pointer_cast<common::Gaussian>(
            result.getUncertaintyEstimate());
    VLOG(1) << " -------- -- ---- Mean estimate: \n"
            << uncertainty->getMean() << "\n --------------- Cov: \n"
            << uncertainty->getCov();
  });
  ds_->startStreaming(0);
}

}  // namespace uncertainty

MAPLAB_UNITTEST_ENTRYPOINT
