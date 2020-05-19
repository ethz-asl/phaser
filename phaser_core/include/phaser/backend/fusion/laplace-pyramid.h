#ifndef PHASER_BACKEND_FUSION_LAPLACE_PYRAMID_H_
#define PHASER_BACKEND_FUSION_LAPLACE_PYRAMID_H_

#include <cstdint>
#include <utility>
#include <vector>

#include <fftw3/fftw3.h>

namespace fusion {

// using complex_t = std::complex<double>;
using complex_t = double[2];

using Pyramid = std::pair<std::vector<complex_t>, std::vector<complex_t>>;

class LaplacePyramid {
 public:
  Pyramid reduce(fftw_complex* coefficients, const uint32_t n_coeffs);
  void expand();
};

}  // namespace fusion

#endif  // PHASER_BACKEND_FUSION_LAPLACE_PYRAMID_H_
