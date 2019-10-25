#include "packlo/backend/alignment/phase-aligner.h"

#include <complex.h> // needs to be included before fftw

#include "igl/histc.h"
#include <fftw3.h>
#include <glog/logging.h>

DEFINE_double(phase_discretize_lower, -80,
    "Specifies the lower bound for the discretization.");
DEFINE_double(phase_discretize_upper, 80,
    "Specifies the upper bound for the discretization.");
DEFINE_double(phase_n_voxels, 161,
    "Specifies the number of voxels for the discretization.");

namespace alignment {

PhaseAligner::PhaseAligner() : 
    n_voxels_(FLAGS_phase_n_voxels 
        * FLAGS_phase_n_voxels 
         * FLAGS_phase_n_voxels) {
  
  // Allocate memory for the FFT and IFFT.
  F_ = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * n_voxels_);
  G_ = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * n_voxels_);
  C_ = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * n_voxels_);
  c_ = new double[n_voxels_];

  // Allocate memory for the function signals in the time domain.
  f_ = Eigen::VectorXd::Zero(n_voxels_);
  g_ = Eigen::VectorXd::Zero(n_voxels_);
  hist_ = Eigen::VectorXd::Zero(n_voxels_);

  // Create the FFTW plans for two FFTs and one IFFT.
  f_plan_ = fftw_plan_dft_r2c_3d(FLAGS_phase_n_voxels,
      FLAGS_phase_n_voxels, FLAGS_phase_n_voxels,
      f_.data(), F_, FFTW_ESTIMATE);

  g_plan_ = fftw_plan_dft_r2c_3d(FLAGS_phase_n_voxels, 
      FLAGS_phase_n_voxels, FLAGS_phase_n_voxels,
      g_.data(), G_, FFTW_ESTIMATE);

  c_plan_ = fftw_plan_dft_c2r_3d(FLAGS_phase_n_voxels,
      FLAGS_phase_n_voxels, FLAGS_phase_n_voxels,
      C_, c_, FFTW_ESTIMATE);
}

PhaseAligner::~PhaseAligner() {
  fftw_free(F_);
  fftw_free(G_);
  fftw_free(C_);

  fftw_destroy_plan(f_plan_);
  fftw_destroy_plan(g_plan_);
  fftw_destroy_plan(c_plan_);

  fftw_cleanup();
  delete [] c_;
}

void PhaseAligner::alignRegistered(
    const model::PointCloud& cloud_prev, 
    const std::vector<model::FunctionValue>&, 
    const model::PointCloud& cloud_reg,
    const std::vector<model::FunctionValue>&, 
    common::Vector_t* xyz) {
  CHECK(xyz);
  discretizePointcloud(cloud_prev, f_, hist_);
  discretizePointcloud(cloud_reg, g_, hist_);

  // Perform the two FFTs on the discretized signals.
  fftw_execute(f_plan_);
  fftw_execute(g_plan_);

  // Correlate the signals in the frequency domain.
  for (uint32_t i = 0u; i < n_voxels_; ++i) {
    C_[i][0] = F_[i][0] * G_[i][0] - F_[i][1] * (-G_[i][1]); 
    C_[i][1] = F_[i][0] * (-G_[i][1]) + F_[i][1] * G_[i][0];
  }

  // Perform the IFFT on the correlation tensor.
  fftw_execute(c_plan_);

  // Find the index that maximizes the correlation.
  const auto max_corr = std::max_element(c_, c_+n_voxels_);
  const int max = std::distance(c_, max_corr);
  VLOG(1) << "Found max correlation at " << max 
    << " with the value :" << *max_corr;

  std::array<uint16_t, 3> max_xyz = ind2sub(max, FLAGS_phase_n_voxels, 
      FLAGS_phase_n_voxels);
  (*xyz)(0) = computeTranslationFromIndex(static_cast<double>(max_xyz[0]));
  (*xyz)(1) = computeTranslationFromIndex(static_cast<double>(max_xyz[1]));
  (*xyz)(2) = computeTranslationFromIndex(static_cast<double>(max_xyz[2]));
}

void PhaseAligner::discretizePointcloud(
    const model::PointCloud& cloud, Eigen::VectorXd& f, 
    Eigen::VectorXd& hist) const {
  Eigen::MatrixXf data = cloud.getRawCloud()->getMatrixXfMap();
  Eigen::VectorXf edges = Eigen::VectorXf::LinSpaced(FLAGS_phase_n_voxels,
      FLAGS_phase_discretize_lower, FLAGS_phase_discretize_upper);

  // Discretize the point cloud using an cartesian grid.
  Eigen::VectorXf x_bins, y_bins, z_bins;
  igl::histc(data.row(0), edges, x_bins);
  igl::histc(data.row(1), edges, y_bins);
  igl::histc(data.row(2), edges, z_bins);

  // Calculate an average function value for each voxel.
  f.setZero();
  hist.setZero();
  const uint32_t n_points = data.cols();
  for (uint16_t i = 0u; i < n_points; ++i) {
    const uint32_t lin_index = sub2ind(x_bins(i), y_bins(i), z_bins(i), 
        FLAGS_phase_n_voxels, FLAGS_phase_n_voxels);
    f(lin_index) = f(lin_index) + cloud.pointAt(i).intensity;
    hist(lin_index) = hist(lin_index) + 1;
  }

  f = f.array() / hist.array();
  f = f.unaryExpr([](double v) { return std::isfinite(v)? v : 0.0; });
}

std::size_t PhaseAligner::sub2ind(const std::size_t i, const std::size_t j, 
    const std::size_t k, const uint32_t rows, const uint32_t cols) 
    const {
  return (i * cols + j) + (rows * cols * k);
}

std::array<uint16_t, 3> PhaseAligner::ind2sub(const int lin_index, 
    const uint32_t rows, const uint32_t cols) const {
  std::array<uint16_t, 3> xyz; 
  xyz[1] = lin_index % cols;
  const int updated_index = lin_index / cols;
  xyz[0] = updated_index % rows;
  xyz[2] = updated_index / rows;
  return xyz;
}

double PhaseAligner::computeTranslationFromIndex(double index) {
  static double n_voxels_half = FLAGS_phase_n_voxels / 2.0;
  static double width = std::abs(FLAGS_phase_discretize_lower) +
    std::abs(FLAGS_phase_discretize_upper);
  if (index <= n_voxels_half) {
    return (index*width) / FLAGS_phase_n_voxels;
  } 
  return (index-FLAGS_phase_n_voxels) * width/ FLAGS_phase_n_voxels;
}

std::vector<double> PhaseAligner::getCorrelation() const {
  return std::vector<double>(c_, c_+n_voxels_);
}

}
// namespace alignment 
