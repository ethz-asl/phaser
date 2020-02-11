#ifndef PACKLO_CONTROLLER_CLOUD_CONTROLLER_H_
#define PACKLO_CONTROLLER_CLOUD_CONTROLLER_H_

#include "phaser/backend/registration/base-registration.h"
#include "phaser/model/point-cloud.h"
#include <string>

namespace controller {

class CloudController {
public:
  CloudController();
  void initializeRegistrationAlgorithm(const std::string& method);
  model::RegistrationResult registerPointCloud(
    const model::PointCloudPtr& target,
    const model::PointCloudPtr& source);
private:
  registration::BaseRegistrationPtr registrator_;
};

}  // namespace controller

#endif  // PACKLO_CONTROLLER_CLOUD_CONTROLLER_H_
