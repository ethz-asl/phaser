#pragma once

#include "packlo/backend/registration/sph-registration.h"

namespace registration {

class SphRegistrationMockTransformed : public SphRegistration {
  public:
    virtual ~SphRegistrationMockTransformed() = default;
    virtual model::RegistrationResult registerPointCloud(
        model::PointCloudPtr cloud_prev, 
        model::PointCloudPtr cloud_cur) override;

  private:
};

} // namespace registration
