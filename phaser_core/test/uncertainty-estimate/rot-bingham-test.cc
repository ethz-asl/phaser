#include <gtest/gtest.h>

#include "phaser/backend/registration/sph-registration.h"
#include "phaser/backend/uncertainty/z-score-eval.h"
#include "phaser/common/data/datasource-ply.h"
#include "phaser/common/metric-utils.h"
#include "phaser/common/test/testing-entrypoint.h"
#include "phaser/common/test/testing-predicates.h"
#include "phaser/distribution/bingham.h"

namespace phaser_core {

class RotBinghamTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    ds_ = std::make_unique<data::DatasourcePly>();
    CHECK_NOTNULL(ds_);
    ds_->setDatasetFolder("./test_clouds/rotation_only/");
    registrator_ =
        std::make_unique<SphRegistration>("phase", "bingham", "gaussian");
    z_score_eval_ =
        dynamic_cast<ZScoreEval*>(&registrator_->getRotEvaluation());
  }

  data::DatasourcePlyPtr ds_;
  SphRegistrationPtr registrator_;
  ZScoreEval* z_score_eval_;
};

TEST_F(RotBinghamTest, LowUncertainty) {
  CHECK(ds_);
  z_score_eval_->getPeakExtraction().getScoreThreshold() = 5.45;
  z_score_eval_->getPeakExtraction().getMaxLag() = 100;

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
    // EXPECT_TRUE(result.foundSolutionForRotation());
    // EXPECT_TRUE(result.foundSolutionForTranslation());

    common::BinghamPtr uncertainty = std::dynamic_pointer_cast<common::Bingham>(
        result.getRotUncertaintyEstimate());
    CHECK_NOTNULL(uncertainty);
    const Eigen::MatrixXd& cov = uncertainty->moment();
    EXPECT_LE(cov.trace(), 1.2);
  });
  ds_->startStreaming(0);
}

}  // namespace phaser_core

MAPLAB_UNITTEST_ENTRYPOINT
