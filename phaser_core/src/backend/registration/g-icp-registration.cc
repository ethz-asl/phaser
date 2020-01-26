#include "packlo/backend/registration/g-icp-registration.h"
#include <glog/logging.h>

namespace registration {

GIcpRegistration::GIcpRegistration() {
  gicp_.setMaximumIterations(100);
  gicp_.setTransformationEpsilon(1e-5);
  gicp_.setRotationEpsilon(1e-5);
  gicp_.setMaximumOptimizerIterations(100);
  gicp_.setCorrespondenceRandomness(50);
}

model::RegistrationResult GIcpRegistration::registerPointCloud(
    model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur) {
  CHECK_NOTNULL(cloud_prev);
  CHECK_NOTNULL(cloud_cur);
  VLOG(1) << " === Starting GICP registration ================================";
  VLOG(1) << "Cloud1: " << cloud_prev->getPlyReadDirectory();
  VLOG(1) << "Cloud2: " << cloud_cur->getPlyReadDirectory();

  common::PointCloud_tPtr transformed(new common::PointCloud_t);
  model::RegistrationResult result;
  try {
    gicp_.setInputSource(cloud_cur->getRawCloud());
    gicp_.setInputTarget(cloud_prev->getRawCloud());
    gicp_.align(*transformed);
  } catch (std::exception& e) {
    result.setRegisteredCloud(cloud_cur);
    result.setGICPResult(Eigen::Matrix4f::Identity(4,4));
    return result;
  }

  model::PointCloudPtr registered
    = std::make_shared<model::PointCloud>(transformed);

  Eigen::Matrix4f T = gicp_.getFinalTransformation();
  result.setRegisteredCloud(registered);
  result.setGICPResult(T);
  VLOG(2) << "G-ICP: \n" << T;

  return result;
}

}  // namespace registration
