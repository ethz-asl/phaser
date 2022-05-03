#ifndef PHASER_BACKEND_UNCERTAINTY_PEAK_BASED_EVAL_H_
#define PHASER_BACKEND_UNCERTAINTY_PEAK_BASED_EVAL_H_

#include <vector>

#include "phaser/backend/uncertainty/base-peak-extraction.h"

namespace phaser_core {

class PeakBasedEval {
 public:
  explicit PeakBasedEval(BasePeakExtractionPtr&& peak_extraction);
  virtual void evaluateCorrelationFromTranslation(
      const std::vector<double>& corr) = 0;

 protected:
  BasePeakExtractionPtr peak_extraction_;
};

}  // namespace phaser_core

#endif  // PHASER_BACKEND_UNCERTAINTY_PEAK_BASED_EVAL_H_
