#include "phaser/backend/fusion/laplace-pyramid.h"

#include <cmath>

#include <glog/logging.h>

namespace fusion {

Pyramid LaplacePyramid::reduce(
    fftw_complex* coefficients, const uint32_t n_coeffs) {
  CHECK_NOTNULL(coefficients);
  // We will reduce the spectrum by half the number of coefficients.
  const uint32_t lower_bound = std::round(n_coeffs / 2.0);
  const uint32_t upper_bound = n_coeffs - lower_bound;
  const uint32_t n_low_pass = upper_bound - lower_bound;
  std::vector<complex_t> coeff_low_pass_full(n_coeffs);
  std::vector<complex_t> coeff_low_pass(n_low_pass);
  std::vector<complex_t> coeff_laplace(n_coeffs);

  uint32_t k = 0;
  for (uint32_t i = lower_bound; i < upper_bound; ++i) {
    coeff_low_pass_full[i][0] = coefficients[i][0];
    coeff_low_pass_full[i][1] = coefficients[i][1];
    coeff_low_pass[k][0] = coefficients[i][0];
    coeff_low_pass[k][1] = coefficients[i][1];
    ++k;
  }

  for (uint32_t i = 0; i < n_coeffs; ++i) {
    coeff_laplace[i][0] = coefficients[i][0] - coeff_low_pass[i][0];
    coeff_laplace[i][1] = coefficients[i][1] - coeff_low_pass[i][1];
  }
  return std::make_pair(std::move(coeff_low_pass), std::move(coeff_laplace));
}

void LaplacePyramid::expand() {}

}  // namespace fusion
