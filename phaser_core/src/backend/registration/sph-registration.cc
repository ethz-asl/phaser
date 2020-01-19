#include "packlo/backend/registration/sph-registration.h"
#include "packlo/backend/alignment/phase-aligner.h"
#include "packlo/backend/alignment/range-based-aligner.h"
#include "packlo/backend/correlation/bingham-peak-based-eval.h"
#include "packlo/backend/correlation/bmm-peak-based-eval.h"
#include "packlo/backend/correlation/gaussian-peak-based-eval.h"
#include "packlo/backend/correlation/gmm-peak-based-eval.h"
#include "packlo/common/rotation-utils.h"
#include "packlo/common/statistic-utils.h"
#include "packlo/common/translation-utils.h"
#include "packlo/visualization/debug-visualizer.h"

#include <algorithm>
#include <glog/logging.h>
#include <iostream>

DEFINE_int32(
    spherical_bandwith, 70,
    "Defines the bandwith used for the spherical registration.");
DEFINE_string(alignment_algorithm, "phase",
    "Sets the algorithm used for the translational alignment.");
DEFINE_string(
    rot_evaluation_algorithm, "bingham",
    "Defines the algorithm used for the evaluation of the rot correlations.");
DEFINE_string(
    pos_evaluation_algorithm, "gaussian",
    "Defines the algorithm used for the evaluation of the pos correlations.");

namespace registration {

SphRegistration::SphRegistration()
    : BaseRegistration("SphRegistration"),
      sampler_(FLAGS_spherical_bandwith),
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
      alignment_algorithm_(alignment_algorithm),
      rot_evaluation_algorithm_(rot_evaluation_algorithm),
      pos_evaluation_algorithm_(pos_evaluation_algorithm) {
  initializeAlgorithms();
}

void SphRegistration::initializeAlgorithms() {
  // Initialize the translational alignment.
  if (alignment_algorithm_ == "phase")
    aligner_ = std::make_unique<alignment::PhaseAligner>();
  else if (alignment_algorithm_ == "averaging")
    aligner_ = std::make_unique<alignment::RangeBasedAligner>();
  else
    LOG(FATAL) << "Unknown alignment algorithm specificed.";
  CHECK_NOTNULL(aligner_);

  // Rotational evaluation
  correlation::BaseEvalPtr rot_eval;
  if (rot_evaluation_algorithm_ == "bingham")
    rot_eval = std::make_unique<correlation::BinghamPeakBasedEval>(
        *aligner_, sph_corr_);
  else if (rot_evaluation_algorithm_ == "bmm")
    rot_eval =
        std::make_unique<correlation::BmmPeakBasedEval>(*aligner_, sph_corr_);
  else
    LOG(FATAL) << "Unknown rot evaluation algorithm specificed: "
               << rot_evaluation_algorithm_;

  // Positional evaluation
  correlation::BaseEvalPtr pos_eval;
  if (pos_evaluation_algorithm_ == "gaussian")
    pos_eval = std::make_unique<correlation::GaussianPeakBasedEval>(
        *aligner_, sph_corr_);
  else if (pos_evaluation_algorithm_ == "gmm")
    pos_eval =
        std::make_unique<correlation::GmmPeakBasedEval>(*aligner_, sph_corr_);
  else
    LOG(FATAL) << "Unknown pos evaluation algorithm specificed: "
               << pos_evaluation_algorithm_;
  correlation_eval_ = std::make_unique<correlation::PhaseCorrelationEval>(
      std::move(rot_eval), std::move(pos_eval));
}

model::RegistrationResult SphRegistration::registerPointCloud(
    model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur) {
  CHECK(cloud_prev);
  CHECK(cloud_cur);
  VLOG(1) << "=== Registering point cloud ====================================";
  cloud_prev->initialize_kd_tree();

  // Register the point cloud.
  /*
  visualization::DebugVisualizer::getInstance().visualizePointCloudDiff(
      *cloud_prev, *cloud_cur);
      */
  model::RegistrationResult result = estimateRotation(cloud_prev, cloud_cur);
  /*
  visualization::DebugVisualizer::getInstance()
    .visualizePointCloudDiff(*cloud_prev, *result.getRegisteredCloud());
    */
  estimateTranslation(cloud_prev, &result);
  /*
  visualization::DebugVisualizer::getInstance()
    .visualizePointCloudDiff(*cloud_prev, *result.getRegisteredCloud());
    */
  return result;
}

model::RegistrationResult SphRegistration::estimateRotation(
    model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur) {
  VLOG(1) << "[SphRegistration] Estimating rotation...";
  cloud_cur->initialize_kd_tree();

  // Correlate point cloud and get uncertainty measure.
  std::array<double, 3> zyz;
  correlatePointcloud(*cloud_prev, *cloud_cur, &zyz);
  common::BaseDistributionPtr rot =
      correlation_eval_->calcRotationUncertainty();
  Eigen::VectorXd b_est =
      common::RotationUtils::ConvertQuaternionToXYZ(rot->getEstimate());
  Eigen::VectorXd corr_est = common::RotationUtils::ConvertZYZtoXYZ(zyz);

  VLOG(1) << "Corr rotation: " << corr_est.transpose();
  VLOG(1) << "Bingham q: " << rot->getEstimate().transpose();
  VLOG(1) << "Bingham rotation: " << b_est.transpose();
  common::RotationUtils::RotateAroundXYZ(
      cloud_cur, b_est(0), b_est(1), b_est(2));

  /*
  common::RotationUtils::RotateAroundXYZ(
      cloud_cur, corr_est(0), corr_est(1), corr_est(2));
      */

  model::RegistrationResult result(std::move(*cloud_cur), std::move(zyz));
  result.setRotUncertaintyEstimate(rot);
  return result;
}

void SphRegistration::estimateTranslation(
    model::PointCloudPtr cloud_prev, model::RegistrationResult* result) {
  VLOG(1) << "[SphRegistration] Estimating translation...";

  common::Vector_t xyz;
  model::PointCloudPtr rot_cloud = result->getRegisteredCloud();
  const double duration_translation_f_ms = common::executeTimedFunction(
      &alignment::BaseAligner::alignRegistered, &(*aligner_), *cloud_prev,
      f_values_, *rot_cloud, h_values_, &xyz);
  CHECK_EQ(xyz.rows(), 3);
  statistics_manager_.emplaceValue(
      kTranslationDurationKey, duration_translation_f_ms);
  common::BaseDistributionPtr pos =
      correlation_eval_->calcTranslationUncertainty();
  Eigen::VectorXd g_est = pos->getEstimate();

  VLOG(1) << "Corr translation: " << xyz.transpose();
  VLOG(1) << "Gaussian translation: " << g_est.transpose();
  VLOG(1) << "Translational alignment took: " << duration_translation_f_ms
    << "ms.";

  common::TranslationUtils::TranslateXYZ(
      rot_cloud, g_est(0), g_est(1), g_est(2));
  /*
common::TranslationUtils::TranslateXYZ(
rot_cloud, xyz(0), xyz(1), xyz(2));
*/
  result->setPosUncertaintyEstimate(pos);
}

void SphRegistration::getStatistics(
    common::StatisticsManager* manager) const noexcept {
  BaseRegistration::getStatistics(manager);
  sph_corr_.getStatistics(manager);
}

void SphRegistration::correlatePointcloud(
    const model::PointCloud& source, const model::PointCloud& target,
    std::array<double, 3>* const zyz) {
  CHECK(zyz);

  const double duration_sample_f_ms = common::executeTimedFunction(
      &common::SphericalSampler::sampleUniformly, &sampler_, source,
      &f_values_);
  const double duration_sample_h_ms = common::executeTimedFunction(
      &common::SphericalSampler::sampleUniformly, &sampler_, target,
      &h_values_);
  CHECK(f_values_.size() == h_values_.size());

  const double duration_correlation_ms = common::executeTimedFunction(
      &backend::SphericalCorrelation::correlateSignals, &sph_corr_, f_values_,
      h_values_, sampler_.getInitializedBandwith(), zyz);

  VLOG(1) << "Registered point cloud.\n"
          << "Sampling took for f and h: [" << duration_sample_f_ms << "ms,"
          << duration_sample_h_ms << "ms]. \n"
          << "Correlation took: " << duration_correlation_ms << "ms.";

  statistics_manager_.emplaceValue(kSampleDurationKey, duration_sample_f_ms);
  statistics_manager_.emplaceValue(kSampleDurationKey, duration_sample_h_ms);
  statistics_manager_.emplaceValue(
      kCorrelationDurationKey,
      duration_correlation_ms + duration_sample_f_ms + duration_sample_h_ms);
  std::vector<double> times =
      statistics_manager_.getValuesForKey(kCorrelationDurationKey);
  std::cout << " rotation timings: \n";
  std::copy(
      times.begin(), times.end(),
      std::ostream_iterator<double>(std::cout, " "));
}

void SphRegistration::setBandwith(const int bandwith) {
  sampler_.initialize(bandwith);
}

correlation::BaseEval& SphRegistration::getRotEvaluation() {
  return correlation_eval_->getRotationEval();
}

correlation::BaseEval& SphRegistration::getPosEvaluation() {
  return correlation_eval_->getPositionEval();
}

}  // namespace registration
