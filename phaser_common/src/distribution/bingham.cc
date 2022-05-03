#include "phaser/distribution/bingham.h"

#include <algorithm>
#include <boost/math/special_functions/bessel.hpp>
#include <cassert>
#include <glog/logging.h>
#include <iostream>
#include <vector>

#include "phaser/common/math-utils.h"
#include "phaser/distribution/bingham-mle.h"
#include "phaser/distribution/bingham-normalization-constant.h"
#include "phaser/distribution/bingham-opt-mle.h"

namespace common {

Bingham::Bingham(const Eigen::VectorXd& vZ, const Eigen::MatrixXd& mM)
    : M(mM),
      Z(vZ),
      S2(MathUtils::computeUnitSphereSurface(1)),
      F(computeF()),
      dF(computeDF()),
      dim(M.rows()) {}

Eigen::VectorXd Bingham::getEstimate() const {
  return mode();
}

Eigen::MatrixXd Bingham::gaussianCovariance(bool angle) {
  if (angle) {
    auto m = mode();
    auto mAngle = std::atan2(m(1), m(0));

    auto res = MathUtils::integral<double>(
        -M_PI / 2, M_PI / 2, 0.01L, [&](double phi) {
          Eigen::MatrixXd tmp(2, 1);
          tmp << std::cos(mAngle + phi), std::sin(mAngle + phi);
          auto density = pdf(tmp);
          auto intRes = density * (std::pow(phi, 2));
          return intRes(0);
        });

    Eigen::MatrixXd eRes(1, 1);
    eRes << 2 * res;
    return eRes;
  } else {
    const uint16_t sample_count = 1000u;
    Eigen::MatrixXd samples;
    sample(&samples, sample_count);
    Eigen::VectorXd m = mode();
    Eigen::MatrixXd zero_samples = samples - m.replicate(samples.rows(), 1);
    return zero_samples * zero_samples.transpose() / sample_count;
  }
}

Eigen::RowVectorXd Bingham::pdf(const Eigen::MatrixXd& xa) const {
  auto diag = Z.asDiagonal();
  Eigen::MatrixXd C = M * diag * M.transpose();

  auto tmp =
      xa.cwiseProduct(C * xa).colwise().sum();  // dxn -> sum over colums -> 1xn
  return 1 / F * tmp.array().exp();             // 1xn
}

Bingham Bingham::multiply(const Bingham& rhs) {
  auto dim = M.rows();
  assert(dim == rhs.M.rows());

  auto diag = Z.asDiagonal();
  auto r_diag = rhs.Z.asDiagonal();
  auto C = M * diag * M.transpose() + rhs.M * r_diag * rhs.M.transpose();
  auto C2 = 0.5 * (C + C.transpose());

  auto eigenV = MathUtils::getSortedEigenVectorsAndValues(C2);

  M = eigenV.first;   // ordered V
  Z = eigenV.second;  // ordered D
  Z = (Z - Z(Z.rows() - 1) * Eigen::VectorXd::Ones(Z.rows()))
          .eval();  // last entry schould be zero.
  F = computeF();
  dF = computeDF();

  return *this;
}

Bingham Bingham::compose(const Bingham& rhs) {
  auto B1 = *this;
  auto B1S = B1.moment();
  auto B2S = rhs.moment();

  if (getDim() == 2) {
    const auto a11 = B1S(0, 0);
    const auto a12 = B1S(0, 1);
    const auto a22 = B1S(1, 1);
    const auto b11 = B1S(0, 0);
    const auto b12 = B1S(0, 1);
    const auto b22 = B1S(1, 1);

    Eigen::MatrixXd S(2, 2);
    S(0, 0) = a11 * b11 - 2 * a12 * b12 + a22 * b22;
    S(0, 1) = a11 * b12 - a22 * b12 - a12 * b22 + a12 * b12;
    S(1, 0) = S(0, 1);
    S(1, 1) = a11 * b22 + 2 * a12 * b12 + a22 * b11;

    return Bingham::fitToMoment(S);
  }
  throw std::runtime_error("unsupported action");
}

Eigen::VectorXd Bingham::mode() const {
  auto n_cols = M.cols();
  return M.col(n_cols - 1);
}

Eigen::MatrixXd Bingham::moment() const {
  Eigen::MatrixXd D = (dF / F).asDiagonal();
  D = (D / D.diagonal().array().sum()).eval();
  auto S = M * D * M.transpose();
  return (S + S.transpose()) / 2;
}

void Bingham::sample(Eigen::MatrixXd* samples, const uint16_t n) const {
  sampleGlover(samples, n);
}

void Bingham::sampleGlover(Eigen::MatrixXd* samples, const uint16_t n) const {
  // The implementation has a bug because it just repeats the previos sample if
  // a sample is rejected.
  //   -> actually that behaviour seems to be coherent with the description of
  //   metropolis-hastings..
  // (http://en.wikipedia.org/wiki/Metropolis-Hastings_algorithm)

  auto burnin = 5;       // = 10 in libbingham
  auto samplerate = 10;  // = 1 in libbingham

  Eigen::VectorXd x = mode();                               // dim x 1
  auto z = Eigen::VectorXd((-1 / (Z.array() - 1)).sqrt());  // dim x 1

  auto target = pdf(x);  // 1x1 row-vector
  auto proposal =
      MathUtils::acgpdf_pcs(x, z, M);  // input dim x 1 -> 1x1 vector

  const auto numSamples = n * samplerate + burnin;
  Eigen::MatrixXd R(numSamples, getDim());
  MathUtils::my_randn(&R);
  auto X2 = Eigen::MatrixXd(
      R.cwiseProduct(z.transpose().replicate(numSamples, 1)) *
      M.transpose());  // sample gaussian (samples x dim)
  auto normalizer = Eigen::VectorXd(
      1 / X2.cwiseProduct(X2).rowwise().sum().array().sqrt());  // samples x 1
  auto norm_X2 = Eigen::MatrixXd(
      X2.cwiseProduct(normalizer.replicate(1, dim)));  // samples x dim

  auto target2 = pdf(
      norm_X2.transpose());  // input: dim x samples -> 1 x samples row-vector
  auto proposal2 = MathUtils::acgpdf_pcs(
      norm_X2, z, M);  // imput: samples x dim -> samples x 1 vector

  auto n_accepts = 0;  // seems to be unused..
  Eigen::MatrixXd X =
      Eigen::MatrixXd::Zero(norm_X2.rows(), norm_X2.cols());  // samples x dim
  double tgt = target(0);
  double prp = proposal(0);
  Eigen::RowVectorXd m =
      x.transpose();  // samples need to be row-vectors in the following. 1xdim

  for (std::size_t i = 0; i < numSamples; ++i) {
    auto a = target2(i) / tgt * prp / proposal2(i);
    double r = MathUtils::my_rand();  // random number in [0, 1)

    if (a > r) {
      m = norm_X2.row(i);
      tgt = target2(i);
      prp = proposal2(i);
      ++n_accepts;
    }
    X.row(i) = m;
  }

  std::size_t idx = 0;
  *samples = Eigen::MatrixXd(
      (numSamples - burnin) / samplerate,
      X.cols());  // retained-samples x dim = nxdim

  for (std::size_t i = burnin; i < numSamples && idx < n; i += samplerate) {
    samples->row(idx++) = X.row(i);
  }

  samples->transposeInPlace();  // dim x n
}

void Bingham::sampleDeterministic(
    Eigen::MatrixXd* samples, Eigen::RowVectorXd* weights, float lambda) {
  if (dim == 2) {
    auto I = Eigen::MatrixXd::Identity(2, 2);
    auto B = Bingham(Z, I);
    auto S = B.moment();

    double alpha = std::asin(std::sqrt(S(0, 0) * 1.5));
    *samples = Eigen::MatrixXd::Zero(3, 2);
    *weights = Eigen::RowVectorXd::Zero(3);
    *samples << 0, 1, std::sin(alpha), std::cos(alpha), -std::sin(alpha),
        std::cos(alpha);
    *samples = (M * samples->transpose()).eval();
    *weights << 1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0;
  } else {
    auto I = Eigen::MatrixXd::Identity(dim, dim);
    auto B = Bingham(Z, I);
    auto S = B.moment();

    *samples =
        Eigen::MatrixXd::Zero(2 * dim - 1, dim);  // will be transposed later..
    *weights = Eigen::RowVectorXd::Zero(2 * dim - 1);
    Eigen::RowVectorXd p = Eigen::RowVectorXd::Zero(dim - 1);
    Eigen::RowVectorXd alpha = Eigen::RowVectorXd::Zero(dim - 1);

    auto end = dim - 1;
    (*samples)(0, end) = 1;  // sample at mode

    auto offset = (1 - lambda) * (S(S.rows() - 1, S.cols() - 1) / end);
    for (std::size_t i = 0u; i < end; ++i) {
      p(i) = S(i, i) + offset;
      alpha(i) = std::asin(std::sqrt(S(i, i) / p(i)));
      (*samples)(2 * i + 1, end) = std::cos(alpha(i));
      (*samples)(2 * i + 2, end) = std::cos(alpha(i));
      (*samples)(2 * i + 1, i) = std::sin(alpha(i));
      (*samples)(2 * i + 2, i) = -std::sin(alpha(i));
      (*weights)(2 * i + 1) = p(i) / 2.0;
      (*weights)(2 * i + 2) = p(i) / 2.0;
    }
    (*weights)(0) = 1.0 - weights->segment(1, 2 * dim - 2).sum();
    *weights = *weights / weights->norm();
    *samples = (M * samples->transpose()).eval();
  }

  return;
}

Bingham Bingham::fitToMoment(
    const Eigen::MatrixXd& S, const std::string& option) {
  auto eigenV = MathUtils::getSortedEigenVectorsAndValues(S);
  Eigen::VectorXd omega = eigenV.second;
  Eigen::MatrixXd mM = eigenV.first;

  double norm = omega.sum();
  omega = omega / norm;  // assure that entries sum to one.
  Eigen::VectorXd res;
  if (option == "default") {
    res = BinghamOptMLE::compute(omega);
  } else if (option == "gaussnewton") {
    res = BinghamMLE::compute(&omega);
  }

  /* This reordering shouldn't be necessary. However, it can
     become necessary as a consequence of numerical errors when
     fitting to moment matrices with almost equal eigenvalues.*/

  const auto eig_dim = res.rows();
  std::vector<size_t> idx(eig_dim);
  std::iota(idx.begin(), idx.end(), 0);
  std::sort(idx.begin(), idx.end(), [&res](size_t i1, size_t i2) {
    return res(i1) < res(i2);
  });

  Eigen::VectorXd Z_ordered(eig_dim);
  Eigen::MatrixXd M_ordered(mM.rows(), mM.cols());
  for (std::size_t i = 0; i < eig_dim; ++i) {
    const auto cur = idx[i];
    Z_ordered(i) = res(cur);
    M_ordered.col(i) = mM.col(cur);
  }

  Z_ordered = (Z_ordered - Z_ordered(Z_ordered.rows() - 1) *
                               Eigen::VectorXd::Ones(Z_ordered.rows()))
                  .eval();

  Bingham bingham(Z_ordered, M_ordered);
  return bingham;
}

Bingham Bingham::fit(
    const Eigen::MatrixXd& samples, const Eigen::RowVectorXd& weights) {
  // assure weights sum to one.
  double norm = weights.sum();
  if (std::abs(norm - 1.0) > 1e-6) {
    std::cout << "Warning: Sample-weights don't sum up to 1.0 but " << norm
              << "!" << std::endl;
  }

  Eigen::MatrixXd C = samples * weights.asDiagonal() * samples.transpose();
  C = 0.5 * (C + C.transpose()).eval();
  Bingham b = fitToMoment(C);
  b.samples_ = samples;
  b.weights_ = weights;
  return b;
}

Bingham Bingham::fit(const Eigen::MatrixXd& samples) {
  int n = samples.cols();

  Eigen::MatrixXd C = samples * samples.transpose() / n;
  C = 0.5 * (C + C.transpose()).eval();
  return fitToMoment(C);
}

double Bingham::computeF() {
  return Bingham::computeF(Z);
}

Eigen::VectorXd Bingham::computeDF() {
  return Bingham::computeDF(Z);
}

double Bingham::computeF(const Eigen::VectorXd& Z) {
  const auto dim = Z.rows();

  if (dim == 2) {
    const auto val = (Z(0) - Z(1)) / 2;
    const double S2 = MathUtils::computeUnitSphereSurface(1);
    return std::exp(Z(1)) * S2 * boost::math::cyl_bessel_i(0.0, val) *
           std::exp(val);
  } else {
    double result[3];
    std::vector<double> z(dim);
    for (int i = 0; i < dim; ++i) {
      z[i] = -Z(i) + 1;
    }
    std::sort(z.begin(), z.end());

    BinghamNormalizationConstant::compute(dim, z.data(), result, nullptr);

    return result[2] * std::exp(1.0);
  }
}

Eigen::VectorXd Bingham::computeDF(const Eigen::VectorXd& Z) {
  const auto dim = Z.rows();
  Eigen::VectorXd dF(dim);

  if (dim == 2) {
    const auto val = (Z(0) - Z(1)) / 2;
    const auto b1 = boost::math::cyl_bessel_i(1, val);
    const auto b0 = boost::math::cyl_bessel_i(0, val);
    const double S2 = MathUtils::computeUnitSphereSurface(1);
    dF(0) = S2 / 2 * (b1 + b0) * std::exp((Z(0) + Z(1)) / 2);
    dF(1) = S2 / 2 * (-b1 + b0) * std::exp((Z(0) + Z(1)) / 2);
  } else {
    for (int i = 0; i < dim; ++i) {
      Eigen::VectorXd ModZ(dim + 2);  // Contains three times the eigenvalue z_i
      ModZ.segment(0, i + 1) = Z.segment(0, i + 1);
      ModZ(i + 1) = Z(i);
      ModZ.segment(i + 2, dim - i) = Z.segment(i, dim - i);

      double result[3];

      std::vector<double> z(dim + 2);
      for (int j = 0; j < dim + 2; ++j) {
        z[j] = -ModZ(j) + 1;
      }
      std::sort(z.begin(), z.end());

      BinghamNormalizationConstant::compute(dim + 2, z.data(), result, nullptr);

      dF(i) = result[2] * std::exp(1.0) / (2.0 * M_PI);
    }
  }
  return dF;
}

}  // namespace common
