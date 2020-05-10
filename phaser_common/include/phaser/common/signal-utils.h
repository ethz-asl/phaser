#ifndef PHASER_COMMON_SIGNAL_UTILS_H_
#define PHASER_COMMON_SIGNAL_UTILS_H_

#include <glog/logging.h>

namespace common {

class SignalUtils {
 public:
  template <typename T_input>
  static void FFTShift(T_input* input, const std::size_t n);
  template <typename T_input>
  static void IFFTShift(T_input* input, const std::size_t n);
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
