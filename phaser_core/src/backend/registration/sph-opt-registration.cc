#include "phaser/backend/registration/sph-opt-registration.h"
#include "phaser/backend/alignment/spatial-correlation-cuda.h"
#include "phaser/backend/alignment/phase-aligner.h"
#include "phaser/backend/alignment/range-based-aligner.h"
#include "phaser/backend/uncertainty/bingham-peak-based-eval.h"
#include "phaser/backend/uncertainty/bmm-peak-based-eval.h"
#include "phaser/backend/uncertainty/gaussian-peak-based-eval.h"
#include "phaser/backend/uncertainty/gmm-peak-based-eval.h"
#include "phaser/common/rotation-utils.h"
#include "phaser/common/translation-utils.h"
#include "phaser/common/statistic-utils.h"

#include <algorithm>
#include <glog/logging.h>
#include <iostream>

namespace registration {

SphOptRegistration::SphOptRegistration()
    : BaseRegistration("SphOptRegistration"),
      sampler_(150) {
  aligner_ = std::make_unique<alignment::PhaseAligner>();

  uncertainty::BaseEvalPtr rot_eval
    = std::make_unique<uncertainty::BinghamPeakBasedEval>(
      *aligner_, sph_corr_);
  uncertainty::BaseEvalPtr pos_eval
    = std::make_unique<uncertainty::GaussianPeakBasedEval>(
      *aligner_, sph_corr_);
  correlation_eval_ = std::make_unique<uncertainty::PhaseCorrelationEval>(
      std::move(rot_eval), std::move(pos_eval));
}

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
  result.getRegisteredCloud()->writeToFile("./");

  return result;
}

model::RegistrationResult SphOptRegistration::estimateRotation(
    model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur) {
  VLOG(1) << "[SphOptRegistration] Estimating rotation...";
  cloud_cur->initialize_kd_tree();

  // Correlate point cloud and get uncertainty measure.
  std::array<double, 3> zyz;
  correlatePointcloud(*cloud_prev, *cloud_cur, &zyz);
  common::BaseDistributionPtr rot =
      correlation_eval_->calcRotationUncertainty();
  Eigen::Vector4d inv = rot->getEstimate();
  inv.block(1,0,3,1) = -inv.block(1,0,3,1);
  Eigen::VectorXd b_est =
      common::RotationUtils::ConvertQuaternionToXYZ(rot->getEstimate());
  Eigen::VectorXd corr_est = common::RotationUtils::ConvertZYZtoXYZ(zyz);

  VLOG(1) << "Corr rotation: " << corr_est.transpose();
  VLOG(1) << "Bingham q: " << rot->getEstimate().transpose();
  VLOG(1) << "Bingham rotation: " << b_est.transpose();
  common::RotationUtils::RotateAroundXYZ(
      cloud_cur, b_est(0), b_est(1), b_est(2));

  model::RegistrationResult result(std::move(*cloud_cur), std::move(zyz));
  result.setRotUncertaintyEstimate(rot);
  result.setRotationCorrelation(sph_corr_.getCorrelation());
  return result;
}

void SphOptRegistration::estimateTranslation(
    model::PointCloudPtr cloud_prev, model::RegistrationResult* result) {
  VLOG(1) << "[SphOptRegistration] Estimating translation...";

  common::Vector_t xyz;
  model::PointCloudPtr rot_cloud = result->getRegisteredCloud();
  const double duration_translation_f_ms = common::executeTimedFunction(
      &alignment::BaseAligner::alignRegistered, &(*aligner_), *cloud_prev,
      f_values_, *rot_cloud, h_values_, &xyz);
  CHECK_EQ(xyz.rows(), 3);
  common::BaseDistributionPtr pos =
      correlation_eval_->calcTranslationUncertainty();
  Eigen::VectorXd g_est = pos->getEstimate();

  VLOG(1) << "Corr translation: " << xyz.transpose();
  VLOG(1) << "Gaussian translation: " << g_est.transpose();
  VLOG(1) << "Translational alignment took: " << duration_translation_f_ms
    << "ms.";

  common::TranslationUtils::TranslateXYZ(
      rot_cloud, g_est(0), g_est(1), g_est(2));
  result->setPosUncertaintyEstimate(pos);
}

void SphOptRegistration::getStatistics(
    common::StatisticsManager* manager) const noexcept {
  BaseRegistration::getStatistics(manager);
  sph_corr_.getStatistics(manager);
}

void SphOptRegistration::correlatePointcloud(
    const model::PointCloud& source, const model::PointCloud& target,
    std::array<double, 3>* const zyz) {
  CHECK(zyz);

  const double duration_sample_f_ms = common::executeTimedFunction(
      &common::SphericalSampler::sampleUniformly, &sampler_, source,
      &f_values_);
  const double duration_sample_h_ms = common::executeTimedFunction(
      &common::SphericalSampler::sampleUniformly, &sampler_, target,
      &h_values_);

  const double duration_correlation_ms = common::executeTimedFunction(
      &backend::SphericalCorrelation::correlateSignals, &sph_corr_, f_values_,
      h_values_, sampler_.getInitializedBandwith(), zyz);

  VLOG(1) << "Registered point cloud.\n"
          << "Sampling took for f and h: [" << duration_sample_f_ms << "ms,"
          << duration_sample_h_ms << "ms]. \n"
          << "Correlation took: " << duration_correlation_ms << "ms.";
}

void SphOptRegistration::setBandwith(const int bandwith) {
  sampler_.initialize(bandwith);
}

uncertainty::BaseEval& SphOptRegistration::getRotEvaluation() {
  return correlation_eval_->getRotationEval();
}

uncertainty::BaseEval& SphOptRegistration::getPosEvaluation() {
  return correlation_eval_->getPositionEval();
}

}  // namespace registration
