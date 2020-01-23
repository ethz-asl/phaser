#include "packlo/backend/registration/g-icp-registration.h"
#include <glog/logging.h>

namespace registration {

model::RegistrationResult GIcpRegistration::registerPointCloud(
    model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur) {

  gicp_.setInputSource(cloud_cur->getRawCloud());
  gicp_.setInputTarget(cloud_prev->getRawCloud());

  common::PointCloud_tPtr transformed(new common::PointCloud_t);
  gicp_.align(*transformed);
  model::PointCloudPtr registered 
    = std::make_shared<model::PointCloud>(transformed);

  model::RegistrationResult result;
  result.setRegisteredCloud(registered);

  VLOG(1) << "G-ICP: \n" << gicp_.getFinalTransformation();

  return result;
}

}  // namespace registration
