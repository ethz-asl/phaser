#include "packlo/common/data/datasource-ply.h"
#include "packlo/backend/registration/sph-registration.h"
#include "packlo/backend/registration/mock/sph-registration-mock-rotated.h"
#include "packlo/common/metric-utils.h"
#include "packlo/common/rotation-utils.h"

#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <random>
#include <chrono>

namespace rotation {

class RotationAlignmentTest : public ::testing::Test {
  public: 
    RotationAlignmentTest() 
        : distribution_(0, M_PI), 
        generator_(std::chrono::system_clock::now()
            .time_since_epoch().count()) {}

  protected:
    virtual void SetUp() {
      ds_ = std::make_unique<data::DatasourcePly>();
    }

    registration::BaseRegistration* initializeRegistration(bool mocked) {
      if (mocked)
        registrator_ = std::make_unique<
          registration::SphRegistrationMockRotated>();
      else
        registrator_ = std::make_unique<registration::SphRegistration>();
      return registrator_.get();
    }

    float getRandomAngle() {
      return distribution_(generator_); 
    }

    data::DatasourcePlyPtr ds_;
    registration::BaseRegistrationPtr registrator_;                             
    std::default_random_engine generator_;
    std::uniform_real_distribution<float> distribution_;
};

TEST_F(RotationAlignmentTest, RotationSelfSingle) {
  CHECK(ds_);
  registration::SphRegistrationMockRotated* reg = dynamic_cast<registration::
    SphRegistrationMockRotated*>(initializeRegistration(true));
  Eigen::Vector3d rot_xyz_rad (M_PI/2.5f, M_PI/2.5f, M_PI/2.5f);
  reg->setRandomRotation(rot_xyz_rad(0), rot_xyz_rad(1), rot_xyz_rad(2));

  model::RegistrationResult result;
  ds_->setDatasetFolder("/home/berlukas/Documents/"
      "workspace/maplab/src/packlo/test/test-data/arche/");
  ds_->subscribeToPointClouds([&] (const model::PointCloudPtr& cloud) {
    CHECK(cloud);
    result = reg->registerPointCloud(cloud, cloud);    
    EXPECT_TRUE(result.foundSolutionForRotation());

    // Convert result to xyz Euler angles and compare it.
    Eigen::Vector3d xyz_rad = common::RotationUtils::ConvertZYZtoXYZ(
        result.getRotation());
    EXPECT_NEAR_EIGEN(-rot_xyz_rad, xyz_rad, 1);
    ASSERT_LE(common::MetricUtils::HausdorffDistance(cloud,
          result.getRegisteredCloud()), 1.0);
  });
  ds_->startStreaming(1);
}

TEST_F(RotationAlignmentTest, RotationSelfAll) {
  CHECK(ds_);
  registration::SphRegistrationMockRotated* reg = dynamic_cast<registration::
    SphRegistrationMockRotated*>(initializeRegistration(true));
  reg->setBandwith(128);

  model::RegistrationResult result;
  ds_->setDatasetFolder("/home/berlukas/Documents/"
      "workspace/maplab/src/packlo/test/test-data/arche/");
  ds_->subscribeToPointClouds([&] (const model::PointCloudPtr& cloud) {
    CHECK(cloud);
    // Define a new random rotation for each incoming cloud.
    Eigen::Vector3d rot_xyz_rad (getRandomAngle(), getRandomAngle(), 
        getRandomAngle());
    reg->setRandomRotation(rot_xyz_rad(0), rot_xyz_rad(1), rot_xyz_rad(2));
    
    // Register the point clouds.
    result = reg->registerPointCloud(cloud, cloud);    
    EXPECT_TRUE(result.foundSolutionForRotation());

    // Check the result.
    ASSERT_LE(common::MetricUtils::HausdorffDistance(cloud,
          result.getRegisteredCloud()), 2.0);
  });
  ds_->startStreaming();
}

TEST_F(RotationAlignmentTest, RotationHighBandwith) {
  CHECK(ds_);
  registration::SphRegistrationMockRotated* reg = dynamic_cast<registration::
    SphRegistrationMockRotated*>(initializeRegistration(true));
  reg->setBandwith(256);

  model::RegistrationResult result;
  ds_->setDatasetFolder("/home/berlukas/Documents/"
      "workspace/maplab/src/packlo/test/test-data/arche/");
  ds_->subscribeToPointClouds([&] (const model::PointCloudPtr& cloud) {
    CHECK(cloud);
    // Define a new random rotation for each incoming cloud.
    Eigen::Vector3d rot_xyz_rad (getRandomAngle(), getRandomAngle(), 
        getRandomAngle());
    reg->setRandomRotation(rot_xyz_rad(0), rot_xyz_rad(1), rot_xyz_rad(2));
    
    // Register the point clouds.
    result = reg->registerPointCloud(cloud, cloud);    
    EXPECT_TRUE(result.foundSolutionForRotation());

    // Check the result.
    ASSERT_LE(common::MetricUtils::HausdorffDistance(cloud,
          result.getRegisteredCloud()), 0.9);
  });
  ds_->startStreaming(1);
}

TEST_F(RotationAlignmentTest, RotationEasy) {
  CHECK(ds_);
  registration::SphRegistration* reg = dynamic_cast<registration::
    SphRegistration*>(initializeRegistration(false));

  model::RegistrationResult result;
  ds_->setDatasetFolder("/home/berlukas/Documents/"
      "workspace/maplab/src/packlo/test/test-data/arche/");
  model::PointCloudPtr prev_cloud = nullptr;
  ds_->subscribeToPointClouds([&] (const model::PointCloudPtr& cloud) {
    CHECK(cloud);
    if (prev_cloud == nullptr) {
      prev_cloud = cloud;
      prev_cloud->initialize_kd_tree();
      return;
    }
   
    // Register the point clouds.
    const float initHausdorff 
      = common::MetricUtils::HausdorffDistance(prev_cloud, cloud);
    cloud->initialize_kd_tree();
    result = reg->estimateRotation(prev_cloud, cloud);    
    EXPECT_TRUE(result.foundSolutionForRotation());

    // Check that the Hausdorff distance decreased after the registration.
    ASSERT_LE(common::MetricUtils::HausdorffDistance(prev_cloud, 
          result.getRegisteredCloud()), initHausdorff);
    prev_cloud = result.getRegisteredCloud();
  });
  ds_->startStreaming();
}

} // namespace rotation

MAPLAB_UNITTEST_ENTRYPOINT