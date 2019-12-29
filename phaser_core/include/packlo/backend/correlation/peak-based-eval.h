#ifndef PACKLO_BACKEND_CORRELATION_PEAK_BASED_EVAL_H_
#define PACKLO_BACKEND_CORRELATION_PEAK_BASED_EVAL_H_

#include <vector>
#include "packlo/backend/correlation/base-peak-extraction.h"

namespace correlation {

class PeakBasedEval {
 public:
  explicit PeakBasedEval(BasePeakExtractionPtr&& peak_extraction);
  virtual void evaluateCorrelationFromTranslation(
      const std::vector<double>& corr) = 0;

 protected:
  BasePeakExtractionPtr peak_extraction_;
};

}  // namespace correlation

#endif  // PACKLO_BACKEND_CORRELATION_PEAK_BASED_EVAL_H_
