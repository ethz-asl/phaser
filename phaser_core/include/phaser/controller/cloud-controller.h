#ifndef PHASER_CONTROLLER_CLOUD_CONTROLLER_H_
#define PHASER_CONTROLLER_CLOUD_CONTROLLER_H_

#include <string>

#include "phaser/backend/registration/base-registration.h"
#include "phaser/model/point-cloud.h"
#include "phaser_pre/cloud-pre-processor.h"

namespace phaser_core {

class CloudController {
 public:
  explicit CloudController(std::string&& method = "sph");
  void initializeRegistrationAlgorithm(const std::string& method);
  model::RegistrationResult registerPointCloud(
      model::PointCloudPtr target, model::PointCloudPtr source);
  void shutdown();

 private:
  BaseRegistrationPtr registrator_;
  preproc::CloudPreProcessor preprocessor_;
};

}  // namespace phaser_core

#endif  // PHASER_CONTROLLER_CLOUD_CONTROLLER_H_
