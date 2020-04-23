#include "phaser/controller/cloud-controller.h"

#include <glog/logging.h>

#include "phaser/backend/registration/sph-opt-registration.h"
#include "phaser/backend/registration/sph-registration.h"

namespace controller {

CloudController::CloudController(std::string&& method) {
  initializeRegistrationAlgorithm(method);
}

void CloudController::initializeRegistrationAlgorithm(
    const std::string& method) {
  if (method == "sph") {
    registrator_ = std::make_unique<registration::SphRegistration>();
  } else if (method == "sph-opt") {
    registrator_ = std::make_unique<registration::SphOptRegistration>();
  } else {
    LOG(FATAL) << "Unknown method specified: " << method;
  }
}

model::RegistrationResult CloudController::registerPointCloud(
    model::PointCloudPtr target, model::PointCloudPtr source) {
  CHECK_NOTNULL(target);
  CHECK_NOTNULL(source);
  CHECK_NOTNULL(registrator_);
  preprocessor_.process(target);
  preprocessor_.process(source);

  return registrator_->registerPointCloud(target, source);
}

}  // namespace controller
