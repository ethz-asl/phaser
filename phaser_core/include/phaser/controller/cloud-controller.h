#ifndef PHASER_CONTROLLER_CLOUD_CONTROLLER_H_
#define PHASER_CONTROLLER_CLOUD_CONTROLLER_H_

#include <string>

#include "phaser/backend/registration/base-registration.h"
#include "phaser/model/point-cloud.h"
#include "phaser_pre/cloud-pre-processor.h"

namespace controller {

class CloudController {
 public:
  explicit CloudController(std::string&& method = "sph");
  void initializeRegistrationAlgorithm(const std::string& method);
  model::RegistrationResult registerPointCloud(
      model::PointCloudPtr target, model::PointCloudPtr source);

 private:
  registration::BaseRegistrationPtr registrator_;
  preproc::CloudPreProcessor preprocessor_;
};

}  // namespace controller

#endif  // PHASER_CONTROLLER_CLOUD_CONTROLLER_H_
