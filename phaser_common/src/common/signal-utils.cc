#include "phaser/common/signal-utils.h"

namespace common {

uint32_t SignalUtils::Sub2Ind(
    const uint32_t i, const uint32_t j, const uint32_t k, const uint32_t rows,
    const uint32_t cols) {
  return (i * cols + j) + (rows * cols * k);
}

void SignalUtils::FFTShift(fftw_complex* input, const std::size_t n) {
  CHECK_NOTNULL(input);
  std::complex<double>* complex_input =
      reinterpret_cast<std::complex<double>*>(input);
  FFTShift(complex_input, n);
}

void SignalUtils::IFFTShift(fftw_complex* input, const std::size_t n) {
  CHECK_NOTNULL(input);
  std::complex<double>* complex_input =
      reinterpret_cast<std::complex<double>*>(input);
  IFFTShift(complex_input, n);
}

}  // namespace common
