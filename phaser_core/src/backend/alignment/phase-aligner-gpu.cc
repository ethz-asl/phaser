#include "phaser/backend/alignment/phase-aligner.h"
#include "phaser/common/point-cloud-utils.h"
#include <complex.h>
#include <cufft.h>

#include <algorithm>
#include <chrono>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "igl/histc.h"

DEFINE_double(
    phase_gpu_discretize_lower, -50,
    "Specifies the lower bound for the discretization.");
DEFINE_double(
    phase_gpu_discretize_upper, 50,
    "Specifies the upper bound for the discretization.");
DEFINE_double(
    phase_gpu_n_voxels, 200,
    "Specifies the number of voxels for the discretization.");
DEFINE_double(
    phase_gpu_batch, 1,
    "");

namespace alignment {

PhaseAligner::PhaseAligner()
    : n_voxels_(
          FLAGS_phase_gpu_n_voxels * FLAGS_phase_gpu_n_voxels
          * FLAGS_phase_gpu_n_voxels) {
  VLOG(1) << "Initializing phase alignment with " << FLAGS_phase_gpu_n_voxels
          << " voxels in [" << FLAGS_phase_gpu_discretize_lower << ", "
          << FLAGS_phase_gpu_discretize_upper << "].";

  // Allocate memory for the FFT and IFFT.
  uint32_t data_size = sizeof(cufftComplex)*n_voxels_*FLAGS_phase_gpu_batch;
  cudaMalloc((void**)&F_, data_size);
  cudaMalloc((void**)&G_, data_size);
  cudaMalloc((void**)&C_, data_size);
  c_ = new double[n_voxels_];

  // Allocate memory for the function signals in the time domain.
  f_ = Eigen::VectorXd::Zero(n_voxels_);
  g_ = Eigen::VectorXd::Zero(n_voxels_);
  hist_ = Eigen::VectorXd::Zero(n_voxels_);

  // Create the FFTW plans for two FFTs and one IFFT.
  f_plan_ = cufftPlan3d(FLAGS_phase_gpu_n_voxels,
      FLAGS_phase_gpu_n_voxels, FLAGS_phase_gpu_n_voxels,
      CUFFT_R2C);
      /*
  g_plan_ = cufftPlan3d(FLAGS_phase_gpu_n_voxels,
      FLAGS_phase_gpu_n_voxels, FLAGS_phase_gpu_n_voxels,
      CUFFT_R2C);
      */
  g_plan_ = cufftPlan3d(FLAGS_phase_gpu_n_voxels,
      FLAGS_phase_gpu_n_voxels, FLAGS_phase_gpu_n_voxels,
      CUFFT_C2R);
}

PhaseAligner::~PhaseAligner() {
  cudaFree(F_);
  cudaFree(G_);
  cudaFree(C_);

  cufftDestroy(f_plan_);
  cufftDestroy(g_plan_);
  cufftDestroy(c_plan_);

  //fftw_cleanup();
  delete [] c_;
}

void PhaseAligner::alignRegistered(
    const model::PointCloud& cloud_prev,
    const std::vector<model::FunctionValue>&,
    const model::PointCloud& cloud_reg,
    const std::vector<model::FunctionValue>&, common::Vector_t* xyz) {
  CHECK(xyz);

  auto start = std::chrono::high_resolution_clock::now();
  discretizePointcloud(cloud_prev, &f_, &hist_);
  discretizePointcloud(cloud_reg, &g_, &hist_);

  // Perform the two FFTs on the discretized signals.
  VLOG(1) << "Performing FFT on the first point cloud.";
  double* d_input;
  cudaMalloc((&d_input), sizeof(double)*n_voxels_);
  cudaMemcpy(d_input, f_.data(), sizeof(double)*n_voxels_, cudaMemcpyHostToDevice);
  cufftExecR2C(f_plan_, d_input, F_);
  VLOG(1) << "Performing FFT on the second point cloud.";
  cudaMemcpy(d_input, g_.data(), sizeof(double)*n_voxels_, cudaMemcpyHostToDevice);
  cufftExecR2C(f_plan_, d_input, G_);

  // Correlate the signals in the frequency domain.
  for (uint32_t i = 0u; i < n_voxels_; ++i) {
    C_[i][0] = F_[i][0] * G_[i][0] - F_[i][1] * (-G_[i][1]);
    C_[i][1] = F_[i][0] * (-G_[i][1]) + F_[i][1] * G_[i][0];
  }

  // Perform the IFFT on the correlation tensor.
  VLOG(1) << "Performing IFFT on correlation.";
  fftw_execute(c_plan_);

  // Find the index that maximizes the correlation.
  const auto max_corr = std::max_element(c_, c_+n_voxels_);
  auto end = std::chrono::high_resolution_clock::now();
  double duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();
  durations_.emplace_back(duration);
  std::cout << "Translation alignment times: \n";
  std::copy(
      durations_.begin(), durations_.end(),
      std::ostream_iterator<double>(std::cout, " "));

  const uint32_t max = std::distance(c_, max_corr);
  std::array<uint32_t, 3> max_xyz =
      ind2sub(max, FLAGS_phase_n_voxels, FLAGS_phase_n_voxels);
  VLOG(1) << "Found max correlation at " << max
          << " with the value:" << *max_corr << " xyz: " << max_xyz[0] << " , "
          << max_xyz[1] << " , " << max_xyz[2];

  (*xyz)(0) = computeTranslationFromIndex(static_cast<double>(max_xyz[0]));
  (*xyz)(1) = computeTranslationFromIndex(static_cast<double>(max_xyz[1]));
  (*xyz)(2) = computeTranslationFromIndex(static_cast<double>(max_xyz[2]));
}

void PhaseAligner::discretizePointcloud(
    const model::PointCloud& cloud, Eigen::VectorXd* f,
    Eigen::VectorXd* hist) const {
  CHECK_NOTNULL(f);
  CHECK_NOTNULL(hist);

  VLOG(1) << "Discretizing point cloud...";
  Eigen::MatrixXf data = cloud.getRawCloud()->getMatrixXfMap();
  Eigen::VectorXf edges = Eigen::VectorXf::LinSpaced(FLAGS_phase_n_voxels,
      FLAGS_phase_discretize_lower, FLAGS_phase_discretize_upper);

  // Discretize the point cloud using an cartesian grid.
  VLOG(1) << "Performing histogram counts.";
  Eigen::VectorXd x_bins, y_bins, z_bins;
  igl::histc(data.row(0), edges, x_bins);
  igl::histc(data.row(1), edges, y_bins);
  igl::histc(data.row(2), edges, z_bins);

  // Calculate an average function value for each voxel.
  f->setZero();
  hist->setZero();
  const uint32_t n_points = data.cols();
  const uint32_t n_f = f->rows();
  for (uint32_t i = 0u; i < n_points; ++i) {
    const uint32_t lin_index = sub2ind(
        x_bins(i), y_bins(i), z_bins(i), FLAGS_phase_n_voxels,
        FLAGS_phase_n_voxels);
    if (lin_index > n_f) {
      continue;
    }
    (*f)(lin_index) = (*f)(lin_index) + cloud.pointAt(i).intensity;
    (*hist)(lin_index) = (*hist)(lin_index) + 1;
  }

  *f = f->array() / hist->array();
  *f = f->unaryExpr([](double v) { return std::isfinite(v) ? v : 0.0; });
  VLOG(1) << "done";
}

uint32_t PhaseAligner::sub2ind(
    const uint32_t i, const uint32_t j, const uint32_t k, const uint32_t rows,
    const uint32_t cols) const {
  return (i * cols + j) + (rows * cols * k);
}
std::array<uint32_t, 3> PhaseAligner::ind2sub(const uint32_t lin_index) const {
  return ind2sub(lin_index, FLAGS_phase_n_voxels, FLAGS_phase_n_voxels);
}

std::array<uint32_t, 3> PhaseAligner::ind2sub(
    const uint32_t lin_index, const uint32_t rows, const uint32_t cols) const {
  std::array<uint32_t, 3> xyz;
  xyz[1] = lin_index % cols;
  const int updated_index = lin_index / cols;
  xyz[0] = updated_index % rows;
  xyz[2] = updated_index / rows;
  /*
  xyz[1] = lin_index % rows;
  const int updated_index = lin_index / rows;
  xyz[0] = updated_index % cols;
  xyz[2] = updated_index / cols;
  */

  return xyz;
}

double PhaseAligner::computeTranslationFromIndex(double index) const {
  static double n_voxels_half = FLAGS_phase_n_voxels / 2.0;
  static double width = std::abs(FLAGS_phase_discretize_lower) +
    std::abs(FLAGS_phase_discretize_upper);
  if (index <= n_voxels_half) {
    return ((index)*width) / FLAGS_phase_n_voxels;
  }
  return (index-FLAGS_phase_n_voxels) * width/ FLAGS_phase_n_voxels;
}

std::vector<double> PhaseAligner::getCorrelation() const {
  return std::vector<double>(c_, c_+n_voxels_);
}

}  // namespace alignment
