#include "phaser/backend/registration/sph-opt-registration.h"

#include <algorithm>
#include <fftw3/fftw3.h>
#include <glog/logging.h>
#include <iostream>

#include "phaser/backend/alignment/phase-aligner.h"
#include "phaser/backend/correlation/spherical-combined-worker.h"
#include "phaser/backend/correlation/spherical-intensity-worker.h"
#include "phaser/backend/correlation/spherical-range-worker.h"
#include "phaser/backend/uncertainty/bingham-peak-based-eval.h"
#include "phaser/backend/uncertainty/gaussian-peak-based-eval.h"
#include "phaser/common/core-gflags.h"
#include "phaser/common/rotation-utils.h"
#include "phaser/common/statistic-utils.h"
#include "phaser/common/translation-utils.h"

namespace phaser_core {

SphOptRegistration::SphOptRegistration()
    : BaseRegistration("SphOptRegistration"),
      bandwidth_(phaser_core::FLAGS_phaser_core_spherical_bandwidth),
      sampler_(phaser_core::FLAGS_phaser_core_spherical_bandwidth) {
  BaseEvalPtr rot_eval = std::make_unique<BinghamPeakBasedEval>();
  BaseEvalPtr pos_eval = std::make_unique<GaussianPeakBasedEval>();
  correlation_eval_ = std::make_unique<PhaseCorrelationEval>(
      std::move(rot_eval), std::move(pos_eval));
  CHECK_NE(fftw_init_threads(), 0);
  fftw_plan_with_nthreads(12);
}
SphOptRegistration::~SphOptRegistration() {}

model::RegistrationResult SphOptRegistration::registerPointCloud(
    model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur) {
  CHECK(cloud_prev);
  CHECK(cloud_cur);
  VLOG(1) << "=== Registering point cloud ====================================";
  VLOG(1) << "Cloud1: " << cloud_prev->getPlyReadDirectory();
  VLOG(1) << "Cloud2: " << cloud_cur->getPlyReadDirectory();
  cloud_prev->initialize_kd_tree();

  // Register the point cloud.
  model::RegistrationResult result = estimateRotation(cloud_prev, cloud_cur);
  estimateTranslation(cloud_prev, &result);
  return result;
}

model::RegistrationResult SphOptRegistration::estimateRotation(
    model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur) {
  VLOG(1) << "[SphOptRegistration] Estimating rotation...";
  // Correlate point cloud and get uncertainty measure.
  std::vector<SphericalCorrelation> correlations =
      correlatePointcloud(cloud_prev, cloud_cur);
  SphericalCorrelation& corr = correlations[0];

  common::BaseDistributionPtr rot =
      correlation_eval_->calcRotationUncertainty(corr);
  Eigen::Vector4d inv = rot->getEstimate();
  inv.block(1, 0, 3, 1) = -inv.block(1, 0, 3, 1);
  Eigen::VectorXd b_est =
      common::RotationUtils::ConvertQuaternionToXYZ(rot->getEstimate());

  VLOG(2) << "Bingham q: " << rot->getEstimate().transpose();
  VLOG(2) << "Bingham rotation: " << b_est.transpose();
  common::RotationUtils::RotateAroundXYZ(
      cloud_cur, b_est(0), b_est(1), b_est(2));

  model::RegistrationResult result(std::move(*cloud_cur));
  result.setRotUncertaintyEstimate(rot);
  result.setRotationCorrelation(corr.getCorrelation());

  return result;
}

void SphOptRegistration::estimateTranslation(
    model::PointCloudPtr cloud_prev, model::RegistrationResult* result) {
  VLOG(1) << "[SphOptRegistration] Estimating translation...";

  model::PointCloudPtr rot_cloud = result->getRegisteredCloud();
  const double duration_translation_f_ms = common::executeTimedFunction(
      &phaser_core::BaseAligner::alignRegistered, &aligner_, *cloud_prev,
      f_values_, *rot_cloud, h_values_);
  common::BaseDistributionPtr pos =
      correlation_eval_->calcTranslationUncertainty(aligner_);
  Eigen::VectorXd g_est = pos->getEstimate();

  VLOG(2) << "Gaussian translation: " << g_est.transpose();
  VLOG(2) << "Translational alignment took: " << duration_translation_f_ms
          << "ms.";

  common::TranslationUtils::TranslateXYZ(
      rot_cloud, g_est(0), g_est(1), g_est(2));
  result->setPosUncertaintyEstimate(pos);
}

void SphOptRegistration::getStatistics(
    common::StatisticsManager* manager) const noexcept {
  BaseRegistration::getStatistics(manager);
}

std::vector<SphericalCorrelation> SphOptRegistration::correlatePointcloud(
    model::PointCloudPtr target, model::PointCloudPtr source) {
  source->initialize_kd_tree();
  target->initialize_kd_tree();

  // Sample the sphere at the grid points.
  std::vector<model::FunctionValue> f_values;
  std::vector<model::FunctionValue> h_values;
  sampler_.sampleUniformly(*target, &f_values);
  sampler_.sampleUniformly(*source, &h_values);

  // Create workers for the spherical correlation.
  // SphericalIntensityWorkerPtr corr_intensity_worker = CHECK_NOTNULL(
  // std::make_shared<SphericalIntensityWorker>(f_values, h_values));
  // SphericalRangeWorkerPtr corr_range_worker =
  // CHECK_NOTNULL(std::make_shared<SphericalRangeWorker>(f_values, h_values));
  SphericalCombinedWorkerPtr corr_combined_worker = CHECK_NOTNULL(
      std::make_shared<SphericalCombinedWorker>(f_values, h_values));

  // Add workers to pool and execute them.
  auto start = std::chrono::high_resolution_clock::now();
  th_pool_.add_worker(corr_combined_worker);
  th_pool_.run_and_wait_all();
  auto end = std::chrono::high_resolution_clock::now();
  VLOG(1) << "Time for rot est: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
                 .count()
          << "ms";
  corr_combined_worker->shutdown();
  return {corr_combined_worker->getCorrelationObject()};
}

void SphOptRegistration::setBandwith(const int bandwith) {
  sampler_.initialize(bandwith);
}

BaseEval& SphOptRegistration::getRotEvaluation() {
  CHECK_NOTNULL(correlation_eval_);
  return correlation_eval_->getRotationEval();
}

BaseEval& SphOptRegistration::getPosEvaluation() {
  CHECK_NOTNULL(correlation_eval_);
  return correlation_eval_->getPositionEval();
}

}  // namespace phaser_core
