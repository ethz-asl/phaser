#include "phaser/backend/alignment/phase-aligner.h"
#include "phaser/backend/correlation/spatial-correlation-cuda.h"
#include "phaser/backend/correlation/spatial-correlation-low-pass.h"
#include "phaser/backend/correlation/spatial-correlation.h"
#include "phaser/common/point-cloud-utils.h"

#include <complex.h>  // needs to be included before fftw

#include <algorithm>
#include <chrono>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <omp.h>
#include "igl/histc.h"

DEFINE_double(
    phase_discretize_lower, -50,
    "Specifies the lower bound for the discretization.");
DEFINE_double(
    phase_discretize_upper, 50,
    "Specifies the upper bound for the discretization.");
DEFINE_double(
    phase_n_voxels, 200,
    "Specifies the number of voxels for the discretization.");

namespace alignment {

PhaseAligner::PhaseAligner()
    : n_voxels_(FLAGS_phase_n_voxels),
      total_n_voxels_(
          FLAGS_phase_n_voxels * FLAGS_phase_n_voxels * FLAGS_phase_n_voxels),
      lower_bound_(FLAGS_phase_discretize_lower),
      upper_bound_(FLAGS_phase_discretize_upper),
      edges_(Eigen::VectorXf::LinSpaced(
          FLAGS_phase_n_voxels, FLAGS_phase_discretize_lower,
          FLAGS_phase_discretize_upper)) {
  VLOG(1) << "Initializing phase alignment with " << FLAGS_phase_n_voxels
          << " voxels in [" << lower_bound_ << ", " << upper_bound_ << "].";
  // Allocate memory for the function signals in the time domain.
  f_intensities_ = Eigen::VectorXd::Zero(total_n_voxels_);
  f_ranges_ = Eigen::VectorXd::Zero(total_n_voxels_);
  g_intensities_ = Eigen::VectorXd::Zero(total_n_voxels_);
  g_ranges_ = Eigen::VectorXd::Zero(total_n_voxels_);
  hist_ = Eigen::VectorXd::Zero(total_n_voxels_);
  spatial_correlation_.reset(
      new correlation::SpatialCorrelationLowPass(n_voxels_));
}

void PhaseAligner::alignRegistered(
    const model::PointCloud& cloud_prev,
    const std::vector<model::FunctionValue>&,
    const model::PointCloud& cloud_reg,
    const std::vector<model::FunctionValue>&) {
  CHECK_NOTNULL(spatial_correlation_);
  discretizePointcloud(cloud_prev, &f_intensities_, &f_ranges_, &hist_);
  discretizePointcloud(cloud_reg, &g_intensities_, &g_ranges_, &hist_);

  double* c = spatial_correlation_->correlateSignals(
      f_intensities_.data(), g_intensities_.data());
  previous_correlation_ = std::vector<double>(c, c + total_n_voxels_);
}

void PhaseAligner::discretizePointcloud(
    const model::PointCloud& cloud, Eigen::VectorXd* f_intensities,
    Eigen::VectorXd* f_ranges, Eigen::VectorXd* hist) const {
  CHECK_NOTNULL(f_intensities);
  CHECK_NOTNULL(f_ranges);
  CHECK_NOTNULL(hist);

  VLOG(1) << "Discretizing point cloud...";
  Eigen::MatrixXf data = cloud.getRawCloud()->getMatrixXfMap();

  // Discretize the point cloud using an cartesian grid.
  VLOG(1) << "Performing histogram counts.";
  Eigen::VectorXd x_bins, y_bins, z_bins;
  igl::histc(data.row(0), edges_, x_bins);
  igl::histc(data.row(1), edges_, y_bins);
  igl::histc(data.row(2), edges_, z_bins);

  // Calculate an average function value for each voxel.
  f_intensities->setZero();
  f_ranges->setZero();
  hist->setZero();
  const uint32_t n_points = data.cols();
  const uint32_t n_f = f_intensities->rows();
#pragma omp parallel for num_threads(4)
  for (uint32_t i = 0u; i < n_points; ++i) {
    const uint32_t lin_index =
        sub2ind(x_bins(i), y_bins(i), z_bins(i), n_voxels_, n_voxels_);
    if (lin_index > n_f) {
      continue;
    }
    (*f_intensities)(lin_index) =
        (*f_intensities)(lin_index) + cloud.pointAt(i).intensity;
    (*f_ranges)(lin_index) = (*f_ranges)(lin_index) + cloud.rangeAt(i);
    (*hist)(lin_index) = (*hist)(lin_index) + 1;
  }
  normalizeSignal(*hist, f_intensities);
  normalizeSignal(*hist, f_ranges);
}

void PhaseAligner::normalizeSignal(
    const Eigen::VectorXd& hist, Eigen::VectorXd* f) const {
  *f = f->array() / hist.array();
  *f = f->unaryExpr([](double v) { return std::isfinite(v) ? v : 0.0; });
}

uint32_t PhaseAligner::sub2ind(
    const uint32_t i, const uint32_t j, const uint32_t k, const uint32_t rows,
    const uint32_t cols) const {
  return (i * cols + j) + (rows * cols * k);
}

std::array<uint32_t, 3> PhaseAligner::ind2sub(const uint32_t lin_index) const {
  return ind2sub(lin_index, n_voxels_, n_voxels_);
}

std::array<uint32_t, 3> PhaseAligner::ind2sub(
    const uint32_t lin_index, const uint32_t rows, const uint32_t cols) const {
  std::array<uint32_t, 3> xyz;
  xyz[1] = lin_index % cols;
  const int updated_index = lin_index / cols;
  xyz[0] = updated_index % rows;
  xyz[2] = updated_index / rows;
  return xyz;
}

double PhaseAligner::computeTranslationFromIndex(double index) const {
  static double n_voxels_half = n_voxels_ / 2.0;
  static double width = std::abs(lower_bound_) + std::abs(upper_bound_);
  if (index <= n_voxels_half) {
    return ((index)*width) / n_voxels_;
  }
  return (index - n_voxels_) * width / n_voxels_;
}

std::vector<double> PhaseAligner::getCorrelation() const {
  return previous_correlation_;
}

uint32_t PhaseAligner::getNumberOfVoxels() const noexcept {
  return n_voxels_;
}

uint32_t PhaseAligner::getLowerBound() const noexcept {
  return lower_bound_;
}

uint32_t PhaseAligner::getUpperBound() const noexcept {
  return upper_bound_;
}

}  // namespace alignment
