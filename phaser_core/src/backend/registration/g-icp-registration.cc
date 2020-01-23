#include "packlo/backend/registration/g-icp-registration.h"
#include <glog/logging.h>

namespace registration {

model::RegistrationResult GIcpRegistration::registerPointCloud(
    model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur) {
  VLOG(1) << " === Starting GICP registration ================================";
  gicp_.setInputSource(cloud_cur->getRawCloud());
  gicp_.setInputTarget(cloud_prev->getRawCloud());

  common::PointCloud_tPtr transformed(new common::PointCloud_t);
  gicp_.align(*transformed);
  model::PointCloudPtr registered
    = std::make_shared<model::PointCloud>(transformed);

  Eigen::Matrix4f T = gicp_.getFinalTransformation();
  model::RegistrationResult result;
  result.setRegisteredCloud(registered);
  result.setGICPResult(T);
  VLOG(2) << "G-ICP: \n" << T;

  return result;
}

}  // namespace registration
