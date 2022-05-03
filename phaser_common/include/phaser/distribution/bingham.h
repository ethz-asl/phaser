/** Definition of Bingham distribution for arbitrary dimensions d
 *
 * Reference goes to:
 * C. Bingham, "An antipodally symmetric distribution on the sphere",
 * The Annals of Statistics, vol. 2, no. 6, pp. 1201-1225, Nov. 1974.
 *
 * Reference implementation:
 * Kurz et al., "Directional Statistics and Filtering Using libDirectional",
 * Journal of Statistical Software, vol. 89, no. 4 (2019): 1-31.
 */
#ifndef PHASER_DISTRIBUTION_BINGHAM_H_
#define PHASER_DISTRIBUTION_BINGHAM_H_

#include <Eigen/Dense>
#include <memory>
#include <string>

#include "phaser/distribution/base-distribution.h"

namespace common {

class Bingham : public BaseDistribution {
 private:
  Eigen::MatrixXd M;      // Rotation matric
  Eigen::VectorXd Z;      // Concentration matrix
  const double S2;        // Second-moment matrix
  double F;               // normalization constant
  Eigen::VectorXd dF;     // Jocabian of normalization constant
  const std::size_t dim;  // Dimensionality

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::VectorXd getEstimate() const override;

 public:
  /**
   * @brief Default constructor for Bingham distribution
   * @param Z (dx1 vector) Vector of diagonal elements of the concentration
   * matrix
   * @param M (dxd matrix) Rotation matrix
   */
  explicit Bingham(const Eigen::VectorXd& vZ, const Eigen::MatrixXd& mM);

  /**
   * @brief Get normlization constant
   * @return Normalization constant of Bingham distribution
   */
  double getF() const noexcept {
    return F;
  }

  /**
   * @brief Get derivative of normalization constant
   * @return Jacobian matrix regarding the normalization constant
   */
  const Eigen::VectorXd& getDF() const noexcept {
    return dF;
  }

  /**
   * @brief Get concentration matrix of Bingham distritbuion represented as
   * vector
   * @return Z(dx1 vector) Vector containing diagonal elements of concentration
   * matrix
   */
  const Eigen::VectorXd& getZ() const noexcept {
    return Z;
  }

  /**
   * @brief Get rotation matrix of Bingham distribution
   * @return M(dxd matrix) Rotation matrix
   */
  const Eigen::MatrixXd& getM() const noexcept {
    return M;
  }

  /**
   * @brief Get dimension of Bingham distribution
   * @return dim(scalar)
   */
  std::size_t getDim() const noexcept {
    return dim;
  }

  /**
   * @brief Compute the covariance to obtain a comparable Gaussian
   *        Note: this is only done with half-sphere because of antiposal
   * symmetry of the distritbution
   * @param angle
   * @return P (1 x 1 matrix for d=2, if angle = true (angle represenation)
   *            d x d matrix for d>=2, if angle = false (vector representation)
   */
  Eigen::MatrixXd gaussianCovariance(bool angle = false);

  /**
   * @brief Evaluate probability density of each column of the input matrix
   * @param xa(dxn matrix) Matrix with each column representing a point in R^d
   * space
   * @return p(1xn vector) Vector of pdf for each point
   */
  Eigen::RowVectorXd pdf(const Eigen::MatrixXd& xa) const;

  /**
   * @brief Computes the product of two Bingham pdfs.
            This method makes use of the fact that the Bingham distribution
            is closed under Bayesian inference. Thus, the product of two
            Bingham pdfs is itself the pdf of a Bingham distribution. This
            method computes the parameters of the resulting distribution.
   * @param rhs Second Bingham distribution to multiply
   * @return Bingham distribution representing this * rhs after renormalization
   */
  Bingham multiply(const Bingham& rhs);

  /**
   * @brief Compose two Bingham distributions using Moment Matching-based
   * approximation The mode of the new distritbution shoule be the quaternion
   * multiplication of the original modes; the uncertainty should be larger than
   * before.
   *
   * @param rh Second Bingham distribution
   * @return Resulted distribution representing the convolution
   */
  Bingham compose(const Bingham& rhs);

  /**
   * @brief Get mode of the Bingham distribution
   * @return Vector representing distribution mode
   */
  Eigen::VectorXd mode() const;

  /**
   * @brief Get the centralized second-oder moment of the Bingham distribution,
   *        which is the same value of its covariance
   * @return S(dxd matrix) Covariance matrix
   */
  Eigen::MatrixXd moment() const;

  /**
   * @brief Random sampling approach for Bingham distribution
   * @param samples Samples to be drawn
   * @param n Numbers of the samples
   */
  void sample(Eigen::MatrixXd* samples, const uint16_t n) const;

  /**
   * @brief Random sampling according to Glover's method from libBingham based
   * on Metropolis-Hastings. Reference goes to:
   *        https://github.com/SebastianRiedel/bingham
   *        See also:
   *        http://en.wikipedia.org/wiki/Metropolis-Hastings_algorithm
   * @param samples Samples to be drawn
   * @param n Number of samples
   */
  void sampleGlover(Eigen::MatrixXd* samples, const uint16_t n) const;

  /**
   * @brief Computes deterministic samples of a Bingham distribution which are
   * the adaption of the UKF. Reference goes to: Igor Gilitschenski, Gerhard
   * Kurz, Simon J. Julier, Uwe D. Hanebeck, Unscented Orientation Estimation
   * Based on the Bingham Distribution IEEE Transactions on Automatic Control,
   * January 2016. It should be noted that the paper gives samples regarding
   * both of the two modes of the distribution. However, the following only
   * samples regarding one mode with samples drawn by mirroring, namely: [s,w] =
   * bd.sampleDeterministic s2 = [s, -s] w2 = [w, w]/2
   * @param samples Samples to be drawn
   * @param weights Corresponding weighting factors for each samples
   * @param lambda(scalar) Weighting parameter in [0,1]
   */
  void sampleDeterministic(
      Eigen::MatrixXd* samples, Eigen::RowVectorXd* weights,
      float lambda = 0.5);

  /**
   * @brief Find a Bingham distribution according to a given second moment
   matrix Reference goes to: Igor Gilitschenski, Gerhard Kurz, Simon J. Julier,
   Uwe D. Hanebeck, Efficient Bingham Filtering based on Saddlepoint
   Approximations Proceedings of the 2014 IEEE International Conference on
   Multisensor Fusion and Information Integration (MFI 2014), Beijing, China,
   September 2014.
   * @param S(dxd matrix) Second moment matrix
   * @return A Bingham distribution
   */
  static Bingham fitToMoment(
      const Eigen::MatrixXd& S, const std::string& option = "default");

  /**
   * @brief Approximating a Bingham distribution with weighted samples
   * @param samples(dxn matrix) Matrix representing a bunch of samples
   * @param weights(1xn vector) Vector containing weights for each sample
   * @return A Bingham distribution
   */
  static Bingham fit(
      const Eigen::MatrixXd& samples, const Eigen::RowVectorXd& weights);

  /**
   * @brief Approximating a Bingham distribution with uniformly weighted samples
   * @param samples(dxn matrix) Matrix representing a bunch of samples
   * @return A Bingham distribution
   */
  static Bingham fit(const Eigen::MatrixXd& samples);

  static double computeF(const Eigen::VectorXd& Z);

  static Eigen::VectorXd computeDF(const Eigen::VectorXd& Z);
  Eigen::MatrixXd samples_;
  Eigen::VectorXd weights_;

 private:
  /**
   * @brief Compute normalization constant
   *        For 2-dimensional case, this is given by the closed-form solution
   * using Bessel function; For higher-dimensional case, reference goes to: Igor
   * Gilitschenski, Gerhard Kurz, Simon J. Julier, Uwe D. Hanebeck, Efficient
   * Bingham Filtering based on Saddlepoint Approximations Proceedings of the
   * 2014 IEEE International Conference on Multisensor Fusion and Information
   * Integration (MFI 2014), Beijing, China, September 2014.
   * @return F(Scalar) Normalization constant
   */
  double computeF();

  /**
   * @brief Compute Jacobian of normliazation constant, namely the parital
   * derivative of normalization constant For 2-dimensional case, this is given
   * by the closed-form solution using Bessel function; For higher-dimensional
   * case, the derivatives of a Bingham normalizing constant can be computed by
   * calculating a normalizing constant of Bingham distributions of higher
   * dimensions. Reference goes to: Igor Gilitschenski, Gerhard Kurz, Simon J.
   * Julier, Uwe D. Hanebeck, Efficient Bingham Filtering based on Saddlepoint
   * Approximations Proceedings of the 2014 IEEE International Conference on
   * Multisensor Fusion and Information Integration (MFI 2014), Beijing, China,
   * September 2014.
   * @return dF(dx1 vector) Partial derivative
   */
  Eigen::VectorXd computeDF();
};

using BinghamPtr = std::shared_ptr<Bingham>;

}  // namespace common

#endif  // PHASER_DISTRIBUTION_BINGHAM_H_
