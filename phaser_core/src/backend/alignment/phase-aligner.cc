#include "phaser/backend/alignment/phase-aligner.h"

#include <algorithm>
#include <chrono>
#include <complex.h>  // needs to be included before fftw
#include <glog/logging.h>
#include <omp.h>
#include <vector>

#include "igl/histc.h"
#include "phaser/backend/correlation/spatial-correlation-laplace.h"
#include "phaser/backend/correlation/spatial-correlation-low-pass.h"
#include "phaser/common/core-gflags.h"
#include "phaser/common/point-cloud-utils.h"
#include "phaser/common/signal-utils.h"

namespace phaser_core {

PhaseAligner::PhaseAligner()
    : n_voxels_(FLAGS_phaser_core_spatial_n_voxels),
      total_n_voxels_(
          FLAGS_phaser_core_spatial_n_voxels *
          FLAGS_phaser_core_spatial_n_voxels *
          FLAGS_phaser_core_spatial_n_voxels),
      lower_bound_(FLAGS_phaser_core_spatial_discretize_lower),
      upper_bound_(FLAGS_phaser_core_spatial_discretize_upper),
      edges_(Eigen::VectorXf::LinSpaced(
          FLAGS_phaser_core_spatial_n_voxels,
          FLAGS_phaser_core_spatial_discretize_lower,
          FLAGS_phaser_core_spatial_discretize_upper)) {
  VLOG(1) << "Initializing phase alignment with " << n_voxels_ << " voxels in ["
          << lower_bound_ << ", " << upper_bound_ << "].";
  // Allocate memory for the function signals in the time domain.
  f_intensities_ = Eigen::VectorXd::Zero(total_n_voxels_);
  f_ranges_ = Eigen::VectorXd::Zero(total_n_voxels_);
  f_reflectivity_ = Eigen::VectorXd::Zero(total_n_voxels_);
  f_ambient_ = Eigen::VectorXd::Zero(total_n_voxels_);
  g_intensities_ = Eigen::VectorXd::Zero(total_n_voxels_);
  g_ranges_ = Eigen::VectorXd::Zero(total_n_voxels_);
  g_reflectivity_ = Eigen::VectorXd::Zero(total_n_voxels_);
  g_ambient_ = Eigen::VectorXd::Zero(total_n_voxels_);
  hist_ = Eigen::VectorXd::Zero(total_n_voxels_);
  spatial_correlation_.reset(new SpatialCorrelationLaplace(
      n_voxels_, FLAGS_phaser_core_spatial_zero_padding));
}

void PhaseAligner::alignRegistered(
    const model::PointCloud& cloud_prev,
    const std::vector<model::FunctionValue>&,
    const model::PointCloud& cloud_reg,
    const std::vector<model::FunctionValue>&) {
  CHECK_NOTNULL(spatial_correlation_);
  std::vector<Eigen::VectorXd*> f = {
      &f_intensities_,
      &f_ranges_,
      &f_reflectivity_,
      &f_ambient_,
  };
  std::vector<Eigen::VectorXd*> g = {
      &g_intensities_, &g_ranges_, &g_reflectivity_, &g_ambient_};
  discretizePointcloud(cloud_prev, f, &hist_);
  discretizePointcloud(cloud_reg, g, &hist_);

  // f.erase(f.begin() + 1);
  // g.erase(g.begin() + 1);
  double* c = spatial_correlation_->correlateSignals(f, g);
  previous_correlation_ =
      std::vector<double>(c, c + spatial_correlation_->getCorrelationSize());
}

void PhaseAligner::discretizePointcloud(
    const model::PointCloud& cloud, const std::vector<Eigen::VectorXd*>& f,
    Eigen::VectorXd* hist) const {
  CHECK_EQ(f.size(), 4u);
  CHECK_NOTNULL(f[0]);
  CHECK_NOTNULL(f[1]);
  CHECK_NOTNULL(f[2]);
  CHECK_NOTNULL(f[3]);
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
  Eigen::VectorXd* f_intensities = f[0];
  Eigen::VectorXd* f_ranges = f[1];
  Eigen::VectorXd* f_reflectivity = f[2];
  Eigen::VectorXd* f_ambient = f[3];
  f_intensities->setZero();
  f_ranges->setZero();
  f_reflectivity->setZero();
  f_ambient->setZero();
  hist->setZero();
  const uint32_t n_points = data.cols();
  const uint32_t n_f = f_intensities->rows();
  // #pragma omp parallel for num_threads(4)
  for (uint32_t i = 0u; i < n_points; ++i) {
    const uint32_t lin_index = common::SignalUtils::Sub2Ind(
        x_bins(i), y_bins(i), z_bins(i), n_voxels_, n_voxels_);
    if (lin_index > n_f) {
      continue;
    }
    (*f_intensities)(lin_index) =
        (*f_intensities)(lin_index) + cloud.pointAt(i).intensity;
    (*f_ranges)(lin_index) = (*f_ranges)(lin_index) + cloud.rangeAt(i);
    if (cloud.hasReflectivityPoints())
      (*f_reflectivity)(lin_index) =
          (*f_reflectivity)(lin_index) + cloud.getReflectivity(i);
    if (cloud.hasAmbientNoisePoints())
      (*f_ambient)(lin_index) =
          (*f_ambient)(lin_index) + cloud.getAmbientNoise(i);
    (*hist)(lin_index) = (*hist)(lin_index) + 1;
  }
  normalizeSignal(*hist, f_intensities);
  normalizeSignal(*hist, f_ranges);
  normalizeSignal(*hist, f_reflectivity);
  normalizeSignal(*hist, f_ambient);
}

void PhaseAligner::normalizeSignal(
    const Eigen::VectorXd& hist, Eigen::VectorXd* f) const {
  *f = f->array() / hist.array();
  *f = f->unaryExpr([](double v) { return std::isfinite(v) ? v : 0.0; });
}

std::vector<double> PhaseAligner::getCorrelation() const {
  return previous_correlation_;
}

uint32_t PhaseAligner::getNumberOfVoxels() const noexcept {
  return n_voxels_ + 2 * spatial_correlation_->getZeroPadding();
}

uint32_t PhaseAligner::getLowerBound() const noexcept {
  return lower_bound_;
}

uint32_t PhaseAligner::getUpperBound() const noexcept {
  return upper_bound_;
}

}  // namespace phaser_core
