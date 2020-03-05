#ifndef PACKLO_BACKEND_REGISTRATION_SPH_CUDA_REGISTRATION_H_
#define PACKLO_BACKEND_REGISTRATION_SPH_CUDA_REGISTRATION_H_

#include "phaser/backend/alignment/base-aligner.h"
#include "phaser/backend/uncertainty/base-eval.h"
#include "phaser/backend/uncertainty/phase-correlation-eval.h"
#include "phaser/backend/correlation/spherical-correlation.h"
#include "phaser/backend/registration/base-registration.h"
#include "phaser/common/spherical-sampler.h"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace registration {

class SphOptRegistration : public BaseRegistration {
 public:
  SphOptRegistration();
  explicit SphOptRegistration(
      std::string&& alignment_algorithm, std::string&& evaluation_algorithm,
      std::string&& pos_evaluation_algorithm);

  virtual ~SphOptRegistration() = default;
  model::RegistrationResult registerPointCloud(
      model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur) override;

  void getStatistics(common::StatisticsManager* manager) const
      noexcept override;

  model::RegistrationResult estimateRotation(
      model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur);

  void estimateTranslation(
      model::PointCloudPtr cloud_prev, model::RegistrationResult* result);

  void setBandwith(const int bandwith);

  correlation::BaseEval& getRotEvaluation();
  correlation::BaseEval& getPosEvaluation();

 protected:
  void correlatePointcloud(
      const model::PointCloud& source, const model::PointCloud& target,
      std::array<double, 3>* const zyz);

  backend::SphericalCorrelation sph_corr_;
  common::SphericalSampler sampler_;
  std::vector<model::FunctionValue> f_values_;
  std::vector<model::FunctionValue> h_values_;
  alignment::BaseAlignerPtr aligner_;
  correlation::PhaseCorrelationEvalPtr correlation_eval_;

  // Statistics
  const std::string kSampleDurationKey = "Sampling";
  const std::string kCorrelationDurationKey = "Correlation";
  const std::string kTranslationDurationKey = "Translation";

 private:
  void initializeAlgorithms();
  std::string alignment_algorithm_;
  std::string rot_evaluation_algorithm_;
  std::string pos_evaluation_algorithm_;
};

using SphCudaRegistrationPtr = std::unique_ptr<SphRegistration>;

}  // namespace registration

#endif  // PACKLO_BACKEND_REGISTRATION_SPH_CUDA_REGISTRATION_H_
