#include "packlo/common/data/datasource-ply.h"
#include "packlo/backend/registration/sph-registration.h"
#include "packlo/backend/registration/mock/sph-registration-mock-translated.h"
#include "packlo/common/metric-utils.h"

#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <random>
#include <chrono>

namespace translation {

class TranslationAlignmentTest : public ::testing::Test {
  public: 
    TranslationAlignmentTest() 
        : distribution_(0, 100), 
        generator_(std::chrono::system_clock::now()
            .time_since_epoch().count()) {}

  protected:
    virtual void SetUp() {
      ds_ = std::make_unique<data::DatasourcePly>();
    }

    registration::BaseRegistration* initializeRegistration(bool mocked) {
      if (mocked)
        registrator_ = std::make_unique<
          registration::SphRegistrationMockTranslated>();
      else
        registrator_ = std::make_unique<registration::SphRegistration>();
      return registrator_.get();
    }

    data::DatasourcePlyPtr ds_;
    registration::BaseRegistrationPtr registrator_;                             
    std::default_random_engine generator_;
    std::uniform_real_distribution<float> distribution_;
};

TEST_F(TranslationAlignmentTest, TranslationSelfSingle) {
  CHECK(ds_);
  registration::SphRegistrationMockTranslated* reg 
    = dynamic_cast<registration::SphRegistrationMockTranslated*>(
        initializeRegistration(true));
  Eigen::Vector3d trans_xyz (12.9f, 33.1f, 21.5f);
  reg->setRandomTranslation(trans_xyz(0), trans_xyz(1), trans_xyz(2));

  model::RegistrationResult result;
  ds_->setDatasetFolder("/home/berlukas/Documents/"
      "workspace/maplab/src/packlo/test/test-data/arche/");
  ds_->subscribeToPointClouds([&] (const model::PointCloudPtr& cloud) {
    CHECK(cloud);
    result = reg->registerPointCloud(cloud, cloud);    
    EXPECT_TRUE(result.foundSolutionForTranslation());

    // Convert result to xyz Euler angles and compare it.
    EXPECT_NEAR_EIGEN(-trans_xyz, result.getTranslation(), 4.0);
    ASSERT_LE(common::MetricUtils::HausdorffDistance(cloud,
          result.getRegisteredCloud()), 5.0);
  });
  ds_->startStreaming(1);
}

TEST_F(TranslationAlignmentTest, TranslationEasy) {
  CHECK(ds_);

}

} // namespace translation

MAPLAB_UNITTEST_ENTRYPOINT
