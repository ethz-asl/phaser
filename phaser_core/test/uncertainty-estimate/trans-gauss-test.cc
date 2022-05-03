#include <gtest/gtest.h>

#include "phaser/backend/registration/sph-registration.h"
#include "phaser/backend/uncertainty/z-score-eval.h"
#include "phaser/common/data/datasource-ply.h"
#include "phaser/common/metric-utils.h"
#include "phaser/common/test/testing-entrypoint.h"
#include "phaser/common/test/testing-predicates.h"
#include "phaser/distribution/gaussian.h"

namespace phaser_core {

class TransGaussTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    ds_ = std::make_unique<data::DatasourcePly>();
    CHECK_NOTNULL(ds_);
    ds_->setDatasetFolder("./test_clouds/translation_only/");
    registrator_ =
        std::make_unique<SphRegistration>("phase", "bingham", "gaussian");
    z_score_eval_ =
        dynamic_cast<ZScoreEval*>(&registrator_->getPosEvaluation());
  }

  data::DatasourcePlyPtr ds_;
  SphRegistrationPtr registrator_;
  ZScoreEval* z_score_eval_;
};

TEST_F(TransGaussTest, LowUncertainty) {
  CHECK(ds_);
  z_score_eval_->getPeakExtraction().getScoreThreshold() = 5.45;

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

    auto rot = result.getRotUncertaintyEstimate();
    CHECK_NOTNULL(rot);
    auto pos = result.getPosUncertaintyEstimate();
    CHECK_NOTNULL(pos);
    common::GaussianPtr uncertainty =
        std::dynamic_pointer_cast<common::Gaussian>(
            result.getPosUncertaintyEstimate());
    CHECK_NOTNULL(uncertainty);
    const Eigen::MatrixXd& cov = uncertainty->getCov();
    EXPECT_LT(cov.trace(), 15);
  });
  ds_->startStreaming(0);
}

}  // namespace phaser_core

MAPLAB_UNITTEST_ENTRYPOINT
