#include "packlo/model/registration-result.h"

namespace model {

RegistrationResult::RegistrationResult() 
    : found_solution_for_rotation_(false), 
    found_solution_for_translation_(false) { }

RegistrationResult::RegistrationResult(model::PointCloud&& reg_cloud, 
    std::array<double, 3>&& rotation) 
    : rotation_(rotation), found_solution_for_rotation_(true) {
  reg_cloud_ = std::make_shared<model::PointCloud>(reg_cloud);
}

RegistrationResult::RegistrationResult(model::PointCloud&& reg_cloud, 
    common::Vector_t&& translation) 
    : translation_(translation), found_solution_for_translation_(true) {
  reg_cloud_ = std::make_shared<model::PointCloud>(reg_cloud);
}

RegistrationResult RegistrationResult::combine(RegistrationResult&& other) {
  reg_cloud_ = other.reg_cloud_;
  found_solution_for_rotation_ |= other.found_solution_for_rotation_;
  found_solution_for_translation_ |= other.found_solution_for_translation_;
  return *this;
}

model::PointCloudPtr RegistrationResult::getRegisteredCloud() const {
  return reg_cloud_;
}

const std::array<double, 3>& RegistrationResult::getRotation() const {
  return rotation_;
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

} // namespace model
