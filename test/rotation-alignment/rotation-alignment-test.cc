#include "maplab-common/test/testing-entrypoint.h"
#include "packlo/common/data/datasource-ply.h"
#include "packlo/backend/registration/sph-registration.h"
#include "packlo/backend/registration/mock/sph-registration-mock-rotated.h"

#include <gtest/gtest.h>
#include <memory>

namespace rotation {

class RotationAlignmentTest : public ::testing::Test {
  protected:
    virtual void SetUp() {
      ds_ = std::make_shared<data::DatasourcePly>();
    }
    registration::BaseRegistration* initializeRegistration(bool mocked) {
      if (mocked)
        registrator_ = std::make_unique<
          registration::SphRegistrationMockRotated>();
      else
        registrator_ = std::make_unique<registration::SphRegistration>();
      return registrator_.get();
    }

    data::DatasourcePtr ds_;
    registration::BaseRegistrationPtr registrator_;                             
};

TEST_F(RotationAlignmentTest, RotationSelf) {
  CHECK(ds_);
  registration::SphRegistrationMockRotated* reg = dynamic_cast<registration::
    SphRegistrationMockRotated*>(initializeRegistration(true));
  const double mock_alpha_rad = M_PI/2.5f;
  const double mock_beta_rad = M_PI/2.5f;
  const double mock_gamma_rad = M_PI/2.5f;
  reg->setRandomRotation(mock_alpha_rad, mock_beta_rad, mock_gamma_rad);

  ds_->subscribeToPointClouds([&] (const model::PointCloudPtr& cloud) {
    CHECK(cloud);
    
  });
}

TEST_F(RotationAlignmentTest, RotationEasy) {
  CHECK(ds_);
  
}

} // namespace rotation

MAPLAB_UNITTEST_ENTRYPOINT
