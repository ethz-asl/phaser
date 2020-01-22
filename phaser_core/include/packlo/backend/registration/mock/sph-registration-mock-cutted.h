#pragma once

#include "packlo/backend/registration/sph-registration.h"
#include <memory>

namespace registration {

class SphRegistrationMockCutted : public SphRegistration {
  public:
    virtual ~SphRegistrationMockCutted() = default;
    virtual model::RegistrationResult registerPointCloud(
        model::PointCloudPtr cloud_prev,
        model::PointCloudPtr cloud_cur) override;

  private:
    model::PointCloudPtr cutPointCloud(common::PointCloud_tPtr& cloud,
        double min, double max, std::string&& dim);
};

} // namespace handler
