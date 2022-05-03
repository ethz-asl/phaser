#include "phaser/common/math-utils.h"

#include <chrono>
#include <numeric>
#include <random>
#include <vector>

double MathUtils::computeUnitSphereSurface(std::size_t d) {
  switch (d) {
    case 0:
      return 2;
    case 1:
      return 2 * M_PI;
    case 2:
      return 4 * M_PI;
    case 3:
      return 2 * M_PI * M_PI;
  }

  return (2 * M_PI / (static_cast<double>(d - 1))) *
         computeUnitSphereSurface(d - 2);
}

Eigen::VectorXd MathUtils::acgpdf_pcs(
    const Eigen::VectorXd& x, const Eigen::VectorXd& z,
    const Eigen::MatrixXd& M) {
  auto X = Eigen::MatrixXd(x.transpose());
  return acgpdf_pcs(X, z, M);
}

Eigen::VectorXd MathUtils::acgpdf_pcs(
    const Eigen::MatrixXd& x, const Eigen::VectorXd& z,
    const Eigen::MatrixXd& M) {
  auto z_diag = Eigen::VectorXd(1 / z.cwiseProduct(z).array());
  auto S_inv = M * z_diag.asDiagonal() * M.transpose();
  auto d = x.cols();
  double z_prod = z.prod();
  auto P_val = Eigen::MatrixXd(1, 1);
  P_val << 1 / (z_prod *
                computeUnitSphereSurface(
                    d - 1));  // if x has 4 elements, compute surface of S^3.

  auto product = (x * S_inv);
  auto md = product.cwiseProduct(x).rowwise().sum();  // mahalanobis distance
  auto dist = Eigen::VectorXd(md.array().pow(-d / 2));

  return P_val.replicate(x.rows(), 1).cwiseProduct(dist);
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd>
MathUtils::getSortedEigenVectorsAndValues(const Eigen::MatrixXd& M) {
  Eigen::EigenSolver<Eigen::MatrixXd> es(M);
  Eigen::VectorXd V = es.eigenvalues().real();
  Eigen::MatrixXd D = es.eigenvectors().real();

  const auto eig_dim = V.rows();
  std::vector<size_t> idx(eig_dim);
  std::iota(idx.begin(), idx.end(), 0);
  std::sort(idx.begin(), idx.end(), [&V](size_t i1, size_t i2) {
    return V(i1) < V(i2);
  });

  Eigen::VectorXd V_ordered(eig_dim);
  Eigen::MatrixXd D_ordered(D.rows(), D.cols());
  for (std::size_t i = 0; i < eig_dim; ++i) {
    const auto cur = idx[i];
    V_ordered(i) = V(cur);
    D_ordered.col(i) = D.col(cur);
  }
  return std::make_pair(D_ordered, V_ordered);
}

std::default_random_engine MathUtils::generator(
    (unsigned)std::chrono::system_clock::now().time_since_epoch().count());

double MathUtils::my_rand() {
  std::uniform_real_distribution<double> distribution(0.0, 1.0);
  return distribution(generator);
}

void MathUtils::my_randn(Eigen::MatrixXd* standardNormalSamples) {
  std::size_t rows = standardNormalSamples->rows();
  std::size_t cols = standardNormalSamples->cols();

  std::normal_distribution<double> distribution(0.0, 1.0);
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      (*standardNormalSamples)(i, j) =
          static_cast<double>(distribution(generator));
    }
  }
}
