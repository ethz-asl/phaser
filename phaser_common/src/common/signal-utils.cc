#include "phaser/common/signal-utils.h"

namespace common {

uint32_t SignalUtils::Sub2Ind(
    const uint32_t i, const uint32_t j, const uint32_t k, const uint32_t rows,
    const uint32_t cols) {
  return (i * cols + j) + (rows * cols * k);
}

std::array<uint32_t, 3> SignalUtils::Ind2Sub(
    const uint32_t lin_index, const uint32_t rows, const uint32_t cols) {
  std::array<uint32_t, 3> xyz;
  xyz[1] = lin_index % cols;
  const int updated_index = lin_index / cols;
  xyz[0] = updated_index % rows;
  xyz[2] = updated_index / rows;
  return xyz;
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
