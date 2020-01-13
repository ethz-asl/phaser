#include "packlo/backend/correlation/base-eval.h"

namespace correlation {

BaseEval::BaseEval(
    const alignment::BaseAligner& aligner,
    const backend::SphericalCorrelation& sph)
    : aligner_(aligner), sph_(sph) {}

}  // namespace correlation
