#include "phaser/backend/registration/sph-registration.h"

#include <algorithm>
#include <glog/logging.h>
#include <iostream>

#include "phaser/backend/alignment/phase-aligner.h"
#include "phaser/backend/uncertainty/bingham-peak-based-eval.h"
#include "phaser/backend/uncertainty/bmm-peak-based-eval.h"
#include "phaser/backend/uncertainty/gaussian-peak-based-eval.h"
#include "phaser/backend/uncertainty/gmm-peak-based-eval.h"
#include "phaser/common/rotation-utils.h"
#include "phaser/common/statistic-utils.h"
#include "phaser/common/translation-utils.h"

DEFINE_int32(
    spherical_bandwith, 75,
    "Defines the bandwith used for the spherical registration.");
DEFINE_string(
    alignment_algorithm, "phase",
    "Sets the algorithm used for the translational alignment.");
DEFINE_string(
    rot_evaluation_algorithm, "bingham",
    "Defines the algorithm used for the evaluation of the rot correlations.");
DEFINE_string(
    pos_evaluation_algorithm, "gaussian",
    "Defines the algorithm used for the evaluation of the pos correlations.");
DEFINE_bool(refine_rot_x, true, "Perform a rotation over x.");
DEFINE_bool(refine_rot_y, true, "Perform a rotation over y.");
DEFINE_bool(refine_rot_z, true, "Perform a rotation over z.");
DEFINE_bool(estimate_rotation, true, "Esttimates the rotation if true.");
DEFINE_bool(estimate_translation, true, "Esttimates the translation if true.");

namespace phaser_core {

SphRegistration::SphRegistration()
    : BaseRegistration("SphRegistration"),
      sampler_(FLAGS_spherical_bandwith),
      sph_corr_(FLAGS_spherical_bandwith),
      alignment_algorithm_(FLAGS_alignment_algorithm),
      rot_evaluation_algorithm_(FLAGS_rot_evaluation_algorithm),
      pos_evaluation_algorithm_(FLAGS_pos_evaluation_algorithm) {
  initializeAlgorithms();
}
SphRegistration::SphRegistration(
    std::string&& alignment_algorithm, std::string&& rot_evaluation_algorithm,
    std::string&& pos_evaluation_algorithm)
    : BaseRegistration("SphRegistration"),
      sampler_(FLAGS_spherical_bandwith),
      sph_corr_(FLAGS_spherical_bandwith),
      alignment_algorithm_(alignment_algorithm),
      rot_evaluation_algorithm_(rot_evaluation_algorithm),
      pos_evaluation_algorithm_(pos_evaluation_algorithm) {
  initializeAlgorithms();
}

void SphRegistration::initializeAlgorithms() {
  // Rotational evaluation
  BaseEvalPtr rot_eval;
  if (rot_evaluation_algorithm_ == "bingham")
    rot_eval = std::make_unique<BinghamPeakBasedEval>();
  else if (rot_evaluation_algorithm_ == "bmm")
    rot_eval = std::make_unique<BmmPeakBasedEval>();
  else
    LOG(FATAL) << "Unknown rot evaluation algorithm specificed: "
               << rot_evaluation_algorithm_;

  // Positional evaluation
  BaseEvalPtr pos_eval;
  if (pos_evaluation_algorithm_ == "gaussian")
    pos_eval = std::make_unique<GaussianPeakBasedEval>();
  else if (pos_evaluation_algorithm_ == "gmm")
    pos_eval = std::make_unique<GmmPeakBasedEval>();
  else
    LOG(FATAL) << "Unknown pos evaluation algorithm specificed: "
               << pos_evaluation_algorithm_;

  correlation_eval_ = std::make_unique<PhaseCorrelationEval>(
      std::move(rot_eval), std::move(pos_eval));
}

model::RegistrationResult SphRegistration::registerPointCloud(
    model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur) {
  CHECK(cloud_prev);
  CHECK(cloud_cur);
  VLOG(1) << "=== Registering point cloud ====================================";
  VLOG(1) << "Cloud1: " << cloud_prev->getPlyReadDirectory();
  VLOG(1) << "Cloud2: " << cloud_cur->getPlyReadDirectory();
  cloud_prev->initialize_kd_tree();

  // Register the point cloud.
  if (!FLAGS_estimate_rotation)
    LOG(FATAL) << "not yet implemented.";
  model::RegistrationResult result = estimateRotation(cloud_prev, cloud_cur);
  if (FLAGS_estimate_translation)
    estimateTranslation(cloud_prev, &result);

  return result;
}

model::RegistrationResult SphRegistration::estimateRotation(
    model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur) {
  VLOG(1) << "[SphRegistration] Estimating rotation...";
  cloud_cur->initialize_kd_tree();

  // Correlate point cloud and get uncertainty measure.
  correlatePointcloud(*cloud_prev, *cloud_cur);
  common::BaseDistributionPtr rot =
      correlation_eval_->calcRotationUncertainty(sph_corr_);
  Eigen::Vector4d inv = rot->getEstimate();
  inv.block(1, 0, 3, 1) = -inv.block(1, 0, 3, 1);
  Eigen::VectorXd b_est =
      common::RotationUtils::ConvertQuaternionToXYZ(rot->getEstimate());
  if (!FLAGS_refine_rot_x)
    b_est(0) = 0;
  if (!FLAGS_refine_rot_y)
    b_est(1) = 0;
  if (!FLAGS_refine_rot_z)
    b_est(2) = 0;

  VLOG(1) << "Bingham q: " << rot->getEstimate().transpose();
  VLOG(1) << "Bingham rotation: " << b_est.transpose();
  common::RotationUtils::RotateAroundXYZ(
      cloud_cur, b_est(0), b_est(1), b_est(2));

  /*
  common::RotationUtils::RotateAroundXYZ(
      cloud_cur, corr_est(0), corr_est(1), corr_est(2));
      */

  model::RegistrationResult result(std::move(*cloud_cur));
  result.setRotUncertaintyEstimate(rot);
  result.setRotationCorrelation(sph_corr_.getCorrelation());
  return result;
}

void SphRegistration::estimateTranslation(
    model::PointCloudPtr cloud_prev, model::RegistrationResult* result) {
  VLOG(1) << "[SphRegistration] Estimating translation...";

  model::PointCloudPtr rot_cloud = result->getRegisteredCloud();
  const double duration_translation_f_ms = common::executeTimedFunction(
      &phaser_core::BaseAligner::alignRegistered, &aligner_, *cloud_prev,
      f_values_, *rot_cloud, h_values_);
  statistics_manager_.emplaceValue(
      kTranslationDurationKey, duration_translation_f_ms);
  common::BaseDistributionPtr pos =
      correlation_eval_->calcTranslationUncertainty(aligner_);
  Eigen::VectorXd g_est = pos->getEstimate();

  VLOG(1) << "Gaussian translation: " << g_est.transpose();
  VLOG(1) << "Translational alignment took: " << duration_translation_f_ms
          << "ms.";

  common::TranslationUtils::TranslateXYZ(
      rot_cloud, g_est(0), g_est(1), g_est(2));
  result->setPosUncertaintyEstimate(pos);
}

void SphRegistration::getStatistics(
    common::StatisticsManager* manager) const noexcept {
  BaseRegistration::getStatistics(manager);
  sph_corr_.getStatistics(manager);
}

void SphRegistration::correlatePointcloud(
    const model::PointCloud& source, const model::PointCloud& target) {
  const double duration_sample_f_ms = common::executeTimedFunction(
      &common::SphericalSampler::sampleUniformly, &sampler_, source,
      &f_values_);
  const double duration_sample_h_ms = common::executeTimedFunction(
      &common::SphericalSampler::sampleUniformly, &sampler_, target,
      &h_values_);
  // CHECK(f_values_.size() == h_values_.size());

  const double duration_correlation_ms = common::executeTimedFunction(
      &SphericalCorrelation::correlateSignals, &sph_corr_, f_values_,
      h_values_);

  VLOG(1) << "Registered point cloud.\n"
          << "Sampling took for f and h: [" << duration_sample_f_ms << "ms,"
          << duration_sample_h_ms << "ms]. \n"
          << "Correlation took: " << duration_correlation_ms << "ms.";

  statistics_manager_.emplaceValue(kSampleDurationKey, duration_sample_f_ms);
  statistics_manager_.emplaceValue(kSampleDurationKey, duration_sample_h_ms);
  statistics_manager_.emplaceValue(
      kCorrelationDurationKey,
      duration_correlation_ms + duration_sample_f_ms + duration_sample_h_ms);
  /*
  std::vector<double> times =
      statistics_manager_.getValuesForKey(kCorrelationDurationKey);
  std::cout << " rotation timings: \n";
  std::copy(
      times.begin(), times.end(),
      std::ostream_iterator<double>(std::cout, " "));
      */
}

void SphRegistration::setBandwith(const int bandwith) {
  sampler_.initialize(bandwith);
}

BaseEval& SphRegistration::getRotEvaluation() {
  return correlation_eval_->getRotationEval();
}

BaseEval& SphRegistration::getPosEvaluation() {
  return correlation_eval_->getPositionEval();
}

}  // namespace phaser_core
