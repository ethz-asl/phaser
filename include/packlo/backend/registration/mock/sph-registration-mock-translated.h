#pragma once

#include "packlo/backend/registration/sph-registration.h"

namespace registration {

class SphRegistrationMockTranslated : public SphRegistration {
  public:
    virtual ~SphRegistrationMockTranslated() = default;
    virtual model::RegistrationResult registerPointCloud(
        model::PointCloudPtr cloud_prev, 
        model::PointCloudPtr cloud_cur) override;

  private:
     model::PointCloud pertubPointCloud(model::PointCloud &cloud,
       const float x, const float y, const float z);
     
    std::vector<model::FunctionValue> 
         pertubFunctionValues(std::vector<model::FunctionValue>& values, 
         const float x, const float y, const float z);
};

} // namespace registration
