#include "phaser/common/signal-utils.h"

namespace common {

uint32_t SignalUtils::Sub2Ind(
    const uint32_t i, const uint32_t j, const uint32_t k, const uint32_t rows,
    const uint32_t cols) {
  return (i * cols + j) + (rows * cols * k);
}

}  // namespace common
