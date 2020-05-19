#ifndef PHASER_BACKEND_FUSION_LAPLACE_PYRAMID_H_
#define PHASER_BACKEND_FUSION_LAPLACE_PYRAMID_H_

#include <cstdint>
#include <utility>
#include <vector>

#include <fftw3/fftw3.h>

namespace fusion {

// using complex_t = std::complex<double>;
using complex_t = double[2];

using PyramidLevel = std::pair<std::vector<complex_t>, std::vector<complex_t>>;

class LaplacePyramid {
 public:
  explicit LaplacePyramid(const float div = 4.0);
  PyramidLevel reduce(fftw_complex* coefficients, const uint32_t n_coeffs);
  void expand();

 private:
  const float divider_;
};

}  // namespace fusion

#endif  // PHASER_BACKEND_FUSION_LAPLACE_PYRAMID_H_
