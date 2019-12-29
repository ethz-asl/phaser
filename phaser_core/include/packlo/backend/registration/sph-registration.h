#ifndef PACKLO_BACKEND_REGISTRATION_SPH_REGISTRATION_H_
#define PACKLO_BACKEND_REGISTRATION_SPH_REGISTRATION_H_

#include "packlo/common/spherical-sampler.h"
#include "packlo/backend/registration/base-registration.h"
#include "packlo/backend/correlation/spherical-correlation.h"
#include "packlo/backend/correlation/base-eval.h"
#include "packlo/backend/alignment/base-aligner.h"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace registration {

class SphRegistration : public BaseRegistration {
 public:
  SphRegistration();

  virtual ~SphRegistration() = default;
  model::RegistrationResult registerPointCloud(
      model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur) override;

  void getStatistics(common::StatisticsManager* manager) const
      noexcept override;

  model::RegistrationResult estimateRotation(
      model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur);

  model::RegistrationResult estimateTranslation(
      model::PointCloudPtr cloud_prev, model::PointCloudPtr rot_cloud);

  void setBandwith(const int bandwith);

  correlation::BaseEval& getEvaluation();

 protected:
  void correlatePointcloud(
      const model::PointCloud& source, const model::PointCloud& target,
      std::array<double, 3>* const zyz);

  backend::SphericalCorrelation sph_corr_;
  common::SphericalSampler sampler_;
  std::vector<model::FunctionValue> f_values_;
  std::vector<model::FunctionValue> h_values_;
  alignment::BaseAlignerPtr aligner_;
  correlation::BaseEvalPtr eval_;

  // Statistics
  const std::string kSampleDurationKey = "Sampling";
  const std::string kCorrelationDurationKey = "Correlation";
  const std::string kTranslationDurationKey = "Translation";

 private:
  void initializeAlgorithms();
};

using SphRegistrationPtr = std::unique_ptr<SphRegistration>;

}  // namespace registration

#endif  // PACKLO_BACKEND_REGISTRATION_SPH_REGISTRATION_H_
