#include "packlo/common/data/datasource-ply.h"
#include "packlo/backend/registration/sph-registration.h"
#include "packlo/backend/registration/mock/sph-registration-mock-rotated.h"

#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include <pcl/search/kdtree.h>

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

  private:
    Eigen::Vector3d fromRotation(const double r11, const double r12, const double r21,
        const double r31, const double r32) {
      Eigen::Vector3d res;
      res(0) = std::atan2(r31, r32);
      res(1) = std::asin(r21);
      res(2) = std::atan2(r11, r12);
      return res;
    }

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
    Eigen::Vector3d convertZYZtoXYZ(const std::array<double, 3>& zyz) {
      Eigen::Quaterniond q = 
        Eigen::AngleAxisd(zyz[0], Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(zyz[1], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(zyz[2], Eigen::Vector3d::UnitZ());
      const double qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
      return fromRotation(-2*(qy*qz - qw*qx),
                    qw*qw - qx*qx - qy*qy + qz*qz,
                    2*(qx*qz + qw*qy),
                   -2*(qx*qy - qw*qz),
                    qw*qw + qx*qx - qy*qy - qz*qz);
    }

    float getRandomAngle() {
      return distribution_(generator_); 
    }

    double calcHausdorff(const model::PointCloudPtr& cloud_a, 
        const model::PointCloudPtr& cloud_b) {
      // compare A to B
      pcl::search::KdTree<common::Point_t> tree_b;
      tree_b.setInputCloud (cloud_b->getRawCloud());
      float max_dist_a = -std::numeric_limits<float>::max();
      for (const auto &point : cloud_a->getRawCloud()->points)
      {
        std::vector<int> indices (1);
        std::vector<float> sqr_distances (1);

        tree_b.nearestKSearch (point, 1, indices, sqr_distances);
        if (sqr_distances[0] > max_dist_a)
          max_dist_a = sqr_distances[0];
      }

      // compare B to A
      pcl::search::KdTree<common::Point_t> tree_a;
      tree_a.setInputCloud (cloud_a->getRawCloud());
      float max_dist_b = -std::numeric_limits<float>::max();
      for (const auto &point : cloud_b->getRawCloud()->points)
      {
        std::vector<int> indices (1);
        std::vector<float> sqr_distances (1);

        tree_a.nearestKSearch (point, 1, indices, sqr_distances);
        if (sqr_distances[0] > max_dist_b)
          max_dist_b = sqr_distances[0];
      }

      max_dist_a = std::sqrt (max_dist_a);
      max_dist_b = std::sqrt (max_dist_b);

      return std::max (max_dist_a, max_dist_b);
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
    Eigen::Vector3d xyz_rad = convertZYZtoXYZ(result.getRotation());
    EXPECT_NEAR_EIGEN(-rot_xyz_rad, xyz_rad, 1);
    ASSERT_LE(calcHausdorff(cloud, result.getRegisteredCloud()), 1.0);
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
    ASSERT_LE(calcHausdorff(cloud, result.getRegisteredCloud()), 2.0);
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
    ASSERT_LE(calcHausdorff(cloud, result.getRegisteredCloud()), 0.75);
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
    float initHausdorff = calcHausdorff(prev_cloud, cloud);
    cloud->initialize_kd_tree();
    result = reg->estimateRotation(prev_cloud, cloud);    
    EXPECT_TRUE(result.foundSolutionForRotation());

    // Check that the Hausdorff distance decreased after the registration.
    ASSERT_LE(calcHausdorff(prev_cloud, cloud), initHausdorff);

  });
  ds_->startStreaming();
}

} // namespace rotation

MAPLAB_UNITTEST_ENTRYPOINT
