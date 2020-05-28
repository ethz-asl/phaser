#include "phaser/backend/uncertainty/phase-correlation-eval.h"

#include <glog/logging.h>

namespace phaser_core {

PhaseCorrelationEval::PhaseCorrelationEval(
    BaseEvalPtr&& rotation, BaseEvalPtr&& positional)
    : rotation_eval_(std::move(rotation)),
      positional_eval_(std::move(positional)) {}

common::BaseDistributionPtr PhaseCorrelationEval::calcRotationUncertainty(
    const SphericalCorrelation& sph_corr) {
  const uint32_t bw = sph_corr.getBandwidth();
  const std::vector<double> corr = sph_corr.getCorrelation();
  return rotation_eval_->evaluateCorrelationFromRotation(bw, corr);
}

common::BaseDistributionPtr PhaseCorrelationEval::calcTranslationUncertainty(
    const phaser_core::PhaseAligner& aligner) {
  const std::vector<double> corr = aligner.getCorrelation();
  const uint32_t n_voxels = aligner.getNumberOfVoxels();
  const uint32_t lower_bound = aligner.getLowerBound();
  const uint32_t upper_bound = aligner.getUpperBound();
  VLOG(1) << "----------- Computing translation with " << n_voxels << " voxels";
  return positional_eval_->evaluateCorrelationFromTranslation(
      n_voxels, lower_bound, upper_bound, corr);
}

BaseEval& PhaseCorrelationEval::getRotationEval() {
  CHECK_NOTNULL(rotation_eval_);
  return *rotation_eval_;
}

BaseEval& PhaseCorrelationEval::getPositionEval() {
  CHECK_NOTNULL(positional_eval_);
  return *positional_eval_;
}

}  // namespace phaser_core
