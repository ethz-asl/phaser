#ifndef INCLUDE_PACKLO_BACKEND_CORRELATION_BASE_PEAK_EXTRACTION_H_
#define INCLUDE_PACKLO_BACKEND_CORRELATION_BASE_PEAK_EXTRACTION_H_

#include <memory>
#include <vector>

namespace correlation {

class BasePeakExtraction {
  virtual void extractPeaks(const std::vector<double>& corr) = 0;
};

using BasePeakExtractionPtr = std::unique_ptr<BasePeakExtraction>;

}  // namespace correlation

#endif  // INCLUDE_PACKLO_BACKEND_CORRELATION_BASE_PEAK_EXTRACTION_H_
