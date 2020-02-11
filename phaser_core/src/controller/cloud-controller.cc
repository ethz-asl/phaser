#include "phaser/controller/cloud-controller.h"
#include "phaser/backend/registration/sph-registration.h"
#include <glog/logging.h>

namespace controller {

CloudController::CloudController() {
  std::string method = "sph";
  initializeRegistrationAlgorithm(method);
}

void CloudController::initializeRegistrationAlgorithm(
    const std::string& method) {
  if (method == "sph") {
    registrator_ = std::make_unique<registration::SphRegistration>();
  }
}

void CloudController::registerPointCloud(const model::PointCloudPtr& target,
    const model::PointCloudPtr& source) {
  CHECK_NOTNULL(target);
  CHECK_NOTNULL(source);
  CHECK_NOTNULL(registrator_);

  model::RegistrationResult result =
    registrator_->registerPointCloud(target, source);
}

}  // namespace controller
