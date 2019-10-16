#include "packlo/backend/alignment/phase-aligner.h"

#include <complex.h> // needs to be included before fftw

#include "igl/histc.h"
#include <fftw3.h>
#include <glog/logging.h>


DEFINE_double(phase_discretize_lower, -80,
    "Specifies the lower bound for the discretization.");
DEFINE_double(phase_discretize_upper, 80,
    "Specifies the upper bound for the discretization.");
DEFINE_double(phase_n_voxels, 81,
    "Specifies the number of voxels for the discretization.");

namespace alignment {

common::Vector_t PhaseAligner::alignRegistered(
    const model::PointCloud& cloud_prev, 
    const std::vector<model::FunctionValue>& f_prev, 
    const model::PointCloud& cloud_reg,
    const std::vector<model::FunctionValue>& f_reg) {
  Eigen::VectorXd f = discretizePointcloud(cloud_prev);
  fftw_complex *F, *G, *C;

  const uint32_t n_voxels = FLAGS_phase_n_voxels * FLAGS_phase_n_voxels
      * FLAGS_phase_n_voxels;

  // TODO make plan a member
  F = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * n_voxels);
  fftw_plan f_plan = fftw_plan_dft_r2c_3d(FLAGS_phase_n_voxels,
      FLAGS_phase_n_voxels, FLAGS_phase_n_voxels,
      f.data(), F, FFTW_ESTIMATE);

  Eigen::VectorXd g = discretizePointcloud(cloud_reg);
  G = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * n_voxels);
  fftw_plan g_plan = fftw_plan_dft_r2c_3d(FLAGS_phase_n_voxels, 
      FLAGS_phase_n_voxels, FLAGS_phase_n_voxels,
      g.data(), G, FFTW_ESTIMATE);

  // fft signals
  fftw_execute(f_plan);
  fftw_execute(g_plan);

  // correlate
  C = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * n_voxels);
  for (uint32_t i = 0u; i < n_voxels; ++i) {
    C[i][0] = F[i][0] * G[i][0] - F[i][1] * (-G[i][1]); 
    C[i][1] = F[i][0] * (-G[i][1]) + F[i][1] * G[i][0];
  }

  fftw_free(F);
  fftw_free(G);
  fftw_destroy_plan(f_plan);
  fftw_destroy_plan(g_plan);

  double* c;
  c = new double[n_voxels];
  fftw_plan c_plan = fftw_plan_dft_c2r_3d(FLAGS_phase_n_voxels,
      FLAGS_phase_n_voxels, FLAGS_phase_n_voxels,
      C, c, FFTW_ESTIMATE);

  // ifft
  fftw_execute(c_plan);
  int max = std::distance(c, std::max_element(c, c+n_voxels));
  
  std::array<uint16_t, 3> xyz = ind2sub(max, FLAGS_phase_n_voxels, 
      FLAGS_phase_n_voxels);

  fftw_destroy_plan(c_plan);
  fftw_cleanup();
  delete [] c;

  common::Vector_t res(
    computeTranslationFromIndex(static_cast<double>(xyz[0])),
    computeTranslationFromIndex(static_cast<double>(xyz[1])),
    computeTranslationFromIndex(static_cast<double>(xyz[2]))
  );

  return res;
}

Eigen::VectorXd PhaseAligner::discretizePointcloud(
    const model::PointCloud& cloud) const {
  Eigen::MatrixXf data = cloud.getRawCloud()->getMatrixXfMap();
  Eigen::VectorXf edges = Eigen::VectorXf::LinSpaced(FLAGS_phase_n_voxels,
      FLAGS_phase_discretize_lower, FLAGS_phase_discretize_upper);

  // discretize the point cloud using an cartesian grid
  Eigen::VectorXf x_bins, y_bins, z_bins;
  igl::histc(data.row(0), edges, x_bins);
  igl::histc(data.row(1), edges, y_bins);
  igl::histc(data.row(2), edges, z_bins);

  const uint32_t n_points = data.cols();
  const uint32_t n_voxels = FLAGS_phase_n_voxels*FLAGS_phase_n_voxels
      * FLAGS_phase_n_voxels;
  Eigen::VectorXd f = Eigen::VectorXd::Zero(n_voxels);
  Eigen::VectorXd hist = Eigen::VectorXd::Zero(n_voxels);
  for (uint16_t i = 0u; i < n_points; ++i) {
    const uint32_t lin_index = sub2ind(x_bins(i), y_bins(i), z_bins(i), 
        FLAGS_phase_n_voxels, FLAGS_phase_n_voxels);
    f(lin_index) = f(lin_index) + cloud.pointAt(i).intensity;
    hist(lin_index) = hist(lin_index) + 1;
  }

  f = f.array() / hist.array();
  f = f.unaryExpr([](double v) { return std::isfinite(v)? v : 0.0; });
  return f;
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

}
// namespace alignment 
