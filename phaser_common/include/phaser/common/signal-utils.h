#ifndef PHASER_COMMON_SIGNAL_UTILS_H_
#define PHASER_COMMON_SIGNAL_UTILS_H_

#include <algorithm>
#include <array>
#include <complex>
#include <fftw3/fftw3.h>
#include <glog/logging.h>

namespace common {

class SignalUtils {
 public:
  template <typename T_input>
  static void FFTShift(T_input* input, const std::size_t n);
  template <typename T_input>
  static void IFFTShift(T_input* input, const std::size_t n);

  static void FFTShift(fftw_complex* input, const std::size_t n);
  static void IFFTShift(fftw_complex* input, const std::size_t n);

  static uint32_t Sub2Ind(
      const uint32_t i, const uint32_t j, const uint32_t k, const uint32_t rows,
      const uint32_t cols);
  static std::array<uint32_t, 3> Ind2Sub(
      const uint32_t lin_index, const uint32_t rows, const uint32_t cols);
};

template <typename T_input>
void SignalUtils::FFTShift(T_input* input, const std::size_t n) {
  CHECK_NOTNULL(input);
  if (n % 2 == 0) {
    std::rotate(&input[0], &input[n >> 1], &input[n]);
  } else {
    std::rotate(&input[0], &input[(n >> 1) + 1], &input[n]);
  }
}

template <typename T_input>
void SignalUtils::IFFTShift(T_input* input, const std::size_t n) {
  CHECK_NOTNULL(input);
  if (n % 2 == 0) {
    std::rotate(&input[0], &input[n >> 1], &input[n]);
  } else {
    std::rotate(&input[0], &input[(n >> 1)], &input[n]);
  }
}

}  // namespace common

#endif  // PHASER_COMMON_SIGNAL_UTILS_H_
