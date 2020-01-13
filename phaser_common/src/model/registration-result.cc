#include "packlo/model/registration-result.h"
#include "packlo/common/rotation-utils.h"

#include <glog/logging.h>

namespace model {

RegistrationResult::RegistrationResult()
    : found_solution_for_rotation_(true),
      found_solution_for_translation_(true) {}

RegistrationResult::RegistrationResult(
    model::PointCloud&& reg_cloud, std::array<double, 3>&& rotation)
    : rotation_(rotation), found_solution_for_rotation_(true) {
  reg_cloud_ = std::make_shared<model::PointCloud>(reg_cloud);
}

RegistrationResult::RegistrationResult(
    model::PointCloud&& reg_cloud, common::Vector_t&& translation)
    : translation_(translation), found_solution_for_translation_(true) {
  reg_cloud_ = std::make_shared<model::PointCloud>(reg_cloud);
}

RegistrationResult::RegistrationResult(model::PointCloudPtr reg_cloud)
    : reg_cloud_(reg_cloud) {}

RegistrationResult RegistrationResult::combine(RegistrationResult&& other) {
  reg_cloud_ = other.reg_cloud_;
  found_solution_for_rotation_ |= other.found_solution_for_rotation_;
  found_solution_for_translation_ |= other.found_solution_for_translation_;
  // current_state_ = other.current_state_;
  return *this;
}

void RegistrationResult::setRegisteredCloud(model::PointCloudPtr reg_cloud) {
  reg_cloud_ = reg_cloud;
}

model::PointCloudPtr RegistrationResult::getRegisteredCloud() const {
  return reg_cloud_;
}

std::array<double, 3> RegistrationResult::getRotation() const {
  return rotation_;
  common::DualQuaternion dq = current_state_.getCurrentState();
  Eigen::Quaterniond q = dq.getRotation();
  Eigen::Vector3d res = common::RotationUtils::ConvertQuaternionToXYZ(q);
  return {res(0), res(1), res(2)};
}

const common::Vector_t& RegistrationResult::getTranslation() const {
  return translation_;
}

bool RegistrationResult::foundSolution() const {
  return found_solution_for_rotation_ && found_solution_for_translation_;
}

bool RegistrationResult::foundSolutionForRotation() const {
  return found_solution_for_rotation_;
}

bool RegistrationResult::foundSolutionForTranslation() const {
  return found_solution_for_translation_;
}

void RegistrationResult::setRotUncertaintyEstimate(
    const common::BaseDistributionPtr& uncertainty) {
  current_state_.setRotationalDistribution(uncertainty);
}

void RegistrationResult::setPosUncertaintyEstimate(
    const common::BaseDistributionPtr& uncertainty) {
  current_state_.setTranslationalDistribution(uncertainty);
  uncertainty_ = uncertainty;
}

common::BaseDistributionPtr RegistrationResult::getRotUncertaintyEstimate()
    const noexcept {
  return current_state_.getRotationalDistribution();
}

common::BaseDistributionPtr RegistrationResult::getPosUncertaintyEstimate()
    const noexcept {
  if (uncertainty_ != nullptr)
    VLOG(1) << "u not null";
  else
    VLOG(1) << "u  null";
  return current_state_.getTranslationalDistribution();
}

}  // namespace model
