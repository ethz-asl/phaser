#include "phaser/model/registration-result.h"

#include <glog/logging.h>

#include "phaser/common/rotation-utils.h"

namespace model {

RegistrationResult::RegistrationResult()
    : found_solution_for_rotation_(true),
      found_solution_for_translation_(true) {}

RegistrationResult::RegistrationResult(
    model::PointCloud&& reg_cloud, common::Vector_t&& translation)
    : translation_(translation), found_solution_for_translation_(true) {
  reg_cloud_ = std::make_shared<model::PointCloud>(reg_cloud);
}

RegistrationResult::RegistrationResult(model::PointCloudPtr reg_cloud)
    : reg_cloud_(reg_cloud) {}

RegistrationResult::RegistrationResult(model::PointCloud&& reg_cloud) {
  reg_cloud_ = std::make_shared<model::PointCloud>(reg_cloud);
}

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

Eigen::Vector3d RegistrationResult::getRotation() const {
  const common::DualQuaternion dq = current_state_.getCurrentState();
  const Eigen::Quaterniond q = dq.getRotation();
  return common::RotationUtils::ConvertQuaternionToXYZ(q);
}

const common::Vector_t& RegistrationResult::getTranslation() const {
  const common::DualQuaternion dq = current_state_.getCurrentState();
  return dq.getTranslation();
}

Eigen::VectorXd RegistrationResult::getStateAsVec() const {
  const common::DualQuaternion dq = current_state_.getCurrentState();
  return dq.asVec();
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
  return current_state_.getTranslationalDistribution();
}

void RegistrationResult::setRotationCorrelation(
    const std::vector<double>& rot) {
  rotation_correlation_ = rot;
}

const std::vector<double>& RegistrationResult::getRotationCorrelation()
    const noexcept {
  return rotation_correlation_;
}

}  // namespace model
