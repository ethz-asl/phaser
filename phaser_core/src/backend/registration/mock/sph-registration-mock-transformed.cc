#include "packlo/backend/registration/mock/sph-registration-mock-transformed.h"
#include "packlo/common/rotation-utils.h"

#include <glog/logging.h>

namespace registration {

model::RegistrationResult SphRegistrationMockTransformed::registerPointCloud(
    model::PointCloudPtr cloud_prev, 
    model::PointCloudPtr) {
  cloud_prev->initialize_kd_tree();
  return model::RegistrationResult();
}

} // namespace handler
