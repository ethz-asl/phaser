#include "packlo/backend/correlation/gaussian-peak-based-eval.h"
#include "packlo/backend/alignment/phase-aligner.h"
#include "packlo/distribution/gaussian.h"

namespace correlation {

GaussianPeakBasedEval::GaussianPeakBasedEval(
    const alignment::BaseAligner& aligner,
    const backend::SphericalCorrelation& sph)
    : ZScoreEval(aligner, sph) {}

common::BaseDistributionPtr GaussianPeakBasedEval::evaluatePeakBasedCorrelation(
    const alignment::BaseAligner& aligner, const std::set<uint32_t>& signals,
    const std::vector<double>&) const {
  Eigen::VectorXd mean;
  Eigen::MatrixXd cov;
  std::tie(mean, cov) = fitTranslationalNormalDist(aligner, signals);
  return std::make_shared<common::Gaussian>(std::move(mean), std::move(cov));
}

std::pair<Eigen::VectorXd, Eigen::MatrixXd>
GaussianPeakBasedEval::fitTranslationalNormalDist(
    const alignment::BaseAligner& aligner,
    const std::set<uint32_t>& signals) const {
  const alignment::PhaseAligner& phase =
      dynamic_cast<const alignment::PhaseAligner&>(aligner);
  const uint32_t n_signals = signals.size();
  if (n_signals == 0) {
    return std::make_pair(
        Eigen::VectorXd::Zero(3), Eigen::MatrixXd::Identity(3, 3));
  }
  Eigen::ArrayXXd samples(3, n_signals);
  uint32_t i = 0u;

  // Extract translational estimates.
  for (uint32_t signal_idx : signals) {
    std::array<uint16_t, 3> xyz = phase.ind2sub(signal_idx);
    samples(0, i) =
        phase.computeTranslationFromIndex(static_cast<double>(xyz[0]));
    samples(1, i) =
        phase.computeTranslationFromIndex(static_cast<double>(xyz[1]));
    samples(2, i) =
        phase.computeTranslationFromIndex(static_cast<double>(xyz[2]));
    ++i;
  }

  // Calculate mean and covariance.
  Eigen::VectorXd mean = samples.rowwise().mean();
  Eigen::VectorXd var =
      ((samples.colwise() - mean.array()).square().rowwise().sum() /
       (n_signals - 1));
  return std::make_pair(mean, var.asDiagonal());
}

}  // namespace correlation
