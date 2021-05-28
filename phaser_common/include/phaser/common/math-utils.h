/** Defines the hyper sphere distritbuion.
 *  Matlab counterpart can be found in class
 *  AbstractHypersphericalDistribution in libDirecitonal
 */

#ifndef PHASER_COMMON_MATH_UTILS_H_
#define PHASER_COMMON_MATH_UTILS_H_

#include <Eigen/Dense>
#include <cmath>
#include <functional>
#include <random>
#include <utility>

class MathUtils {
 private:
  static std::default_random_engine generator;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  /**
   * @brief Computes surface area of (d-1)-sphere
   *        see http://en.wikipedia.org/wiki/N-sphere#Volume_and_surface_area
   *        dimension = 2 => circle
   *        dimension = 3 => sphere
   * @param dimension of the hypersphere
   * @return surface area(scalar)
   */
  static double computeUnitSphereSurface(std::size_t dimension);

  /**
   * @brief Get pdf given eigenvalues and eigenvector
   * @param x (nx1 vector) Sample
   * @param z (dxd matrix) Sqrt of diagonal matrix containing eigenvalue
   * @param M (dxd matrix) Sqrt of eigenvectors
   * @return
   */
  static Eigen::VectorXd acgpdf_pcs(
      const Eigen::VectorXd& x, const Eigen::VectorXd& z,
      const Eigen::MatrixXd& M);

  /**
   * @brief Get pdf given eigenvalues and eigenvector
   * @param x (dxn vector) Matrix containing samples at each column
   * @param z (dxd matrix) Sqrt of diagonal matrix containing eigenvalue
   * @param M (dxd matrix) Sqrt of eigenvectors
   * @return
   */
  static Eigen::VectorXd acgpdf_pcs(
      const Eigen::MatrixXd& x, const Eigen::VectorXd& z,
      const Eigen::MatrixXd& M);

  /**
   * @brief Get ordered eigenvectors according to eigenvalues
   * @param M (dxd matrix) Covariance matrix to be decomposed
   * @return <V, D> Eigenvalues in ascending order paried with corresponding
   * eigenvectors
   */
  static std::pair<Eigen::MatrixXd, Eigen::VectorXd>
  getSortedEigenVectorsAndValues(const Eigen::MatrixXd& M);

  /**
   * @brief Generate random samples according to standard normal distribution
   * @param standardNormalSamples Samples to be generated
   */
  static void my_randn(Eigen::MatrixXd* standardNormalSamples);

  /**
   * @brief Generate random samples according to uniform distribution between 0
   * and 1
   * @return Uniformly distributed samples in [0,1]
   */
  static double my_rand();

  /** Compute pseudo-inverse of a dense matrix. Reference goes to:
   *  https://gist.github.com/javidcf/25066cf85e71105d57b6
   */
  template <class MatT>
  static Eigen::Matrix<
      typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
  pseudoinverse(
      const MatT& mat,
      typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4});

  /** Compute integral given function between in [a,b] given tolerance tol
   */
  template <typename value_type>
  static value_type integral(
      const value_type a, const value_type b, const value_type tol,
      std::function<value_type(value_type)> func);
};

template <typename value_type>
value_type MathUtils::integral(
    const value_type a, const value_type b, const value_type tol,
    std::function<value_type(value_type)> func) {
  unsigned n = 1U;

  value_type h = (b - a);
  value_type I = (func(a) + func(b)) * (h / 2);

  for (unsigned k = 0U; k < 8U; k++) {
    h /= 2;

    value_type sum(0);
    for (unsigned j = 1U; j <= n; j++) {
      sum += func(a + (value_type((j * 2) - 1) * h));
    }

    const value_type I0 = I;
    I = (I / 2) + (h * sum);

    const value_type ratio = I0 / I;
    const value_type delta = ratio - 1;
    const value_type delta_abs = ((delta < 0) ? -delta : delta);

    if ((k > 1U) && (delta_abs < tol)) {
      break;
    }

    n *= 2U;
  }

  return I;
}

template <class MatT>
Eigen::Matrix<
    typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
MathUtils::pseudoinverse(const MatT& mat, typename MatT::Scalar tolerance) {
  typedef typename MatT::Scalar Scalar;
  auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto& singularValues = svd.singularValues();
  Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
      singularValuesInv(mat.cols(), mat.rows());
  singularValuesInv.setZero();
  for (unsigned int i = 0; i < singularValues.size(); ++i) {
    if (singularValues(i) > tolerance) {
      singularValuesInv(i, i) = Scalar(1) / singularValues(i);
    } else {
      singularValuesInv(i, i) = Scalar(0);
    }
  }
  return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}

#endif  // PHASER_COMMON_MATH_UTILS_H_
