#include "phaser/backend/uncertainty/peak-based-eval.h"

namespace phaser_core {

PeakBasedEval::PeakBasedEval(BasePeakExtractionPtr&& peak_extraction)
    : peak_extraction_(std::move(peak_extraction)) {}

}  // namespace phaser_core
