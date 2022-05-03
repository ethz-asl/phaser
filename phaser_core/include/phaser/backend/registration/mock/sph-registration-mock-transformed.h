#pragma once

#include "phaser/backend/registration/sph-registration.h"

namespace phaser_core {

class SphRegistrationMockTransformed : public SphRegistration {
 public:
  virtual ~SphRegistrationMockTransformed() = default;
  virtual model::RegistrationResult registerPointCloud(
      model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur) override;

 private:
};

}  // namespace phaser_core
