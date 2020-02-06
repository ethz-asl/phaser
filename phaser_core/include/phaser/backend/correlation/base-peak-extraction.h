#ifndef PACKLO_BACKEND_CORRELATION_BASE_PEAK_EXTRACTION_H_
#define PACKLO_BACKEND_CORRELATION_BASE_PEAK_EXTRACTION_H_

#include <memory>
#include <set>
#include <vector>

namespace correlation {

class BasePeakExtraction {
  virtual void extractPeaks(
      const std::vector<double>& corr, std::set<uint32_t>* peaks) = 0;
};

using BasePeakExtractionPtr = std::unique_ptr<BasePeakExtraction>;

}  // namespace correlation

#endif  // PACKLO_BACKEND_CORRELATION_BASE_PEAK_EXTRACTION_H_
