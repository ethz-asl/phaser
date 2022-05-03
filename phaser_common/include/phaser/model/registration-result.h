#ifndef PHASER_MODEL_REGISTRATION_RESULT_H_
#define PHASER_MODEL_REGISTRATION_RESULT_H_

#include <array>
#include <memory>
#include <vector>

#include "phaser/distribution/base-distribution.h"
#include "phaser/model/point-cloud.h"
#include "phaser/model/state.h"

namespace model {

class RegistrationResult {
 public:
  RegistrationResult();
  explicit RegistrationResult(
      model::PointCloud&& reg_cloud, common::Vector_t&& rotation);
  explicit RegistrationResult(model::PointCloudPtr reg_cloud);
  explicit RegistrationResult(model::PointCloud&& reg_cloud);

  RegistrationResult combine(RegistrationResult&& other);

  model::PointCloudPtr getRegisteredCloud() const;
  void setRegisteredCloud(model::PointCloudPtr reg_cloud);

  Eigen::Vector3d getRotation() const;
  const common::Vector_t& getTranslation() const;
  Eigen::VectorXd getStateAsVec() const;

  bool foundSolution() const;
  bool foundSolutionForRotation() const;
  bool foundSolutionForTranslation() const;

  void setRotUncertaintyEstimate(
      const common::BaseDistributionPtr& uncertainty);
  void setPosUncertaintyEstimate(
      const common::BaseDistributionPtr& uncertainty);
  common::BaseDistributionPtr getRotUncertaintyEstimate() const noexcept;
  common::BaseDistributionPtr getPosUncertaintyEstimate() const noexcept;

  void setRotationCorrelation(const std::vector<double>& rot);
  const std::vector<double>& getRotationCorrelation() const noexcept;

 private:
  model::PointCloudPtr reg_cloud_;
  std::array<double, 3> rotation_;
  common::Vector_t translation_;
  bool found_solution_for_rotation_;
  bool found_solution_for_translation_;
  common::BaseDistributionPtr uncertainty_;
  State current_state_;
  std::vector<double> rotation_correlation_;
};

}  // namespace model

#endif  // PHASER_MODEL_REGISTRATION_RESULT_H_
