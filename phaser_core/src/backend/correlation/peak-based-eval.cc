#include "packlo/backend/correlation/peak-based-eval.h"

namespace correlation {

PeakBasedEval::PeakBasedEval(BasePeakExtractionPtr&& peak_extraction)
    : peak_extraction_(std::move(peak_extraction)) {}

}  // namespace correlation
