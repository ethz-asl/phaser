#include "packlo/backend/correlation/z-score-eval.h"

#include <cmath>
#include <cstdint>

namespace correlation {

  void ZScoreEval::evaluateCorrelationFromTranslation(
      const std::vector<double>& corr) {
    std::vector<int> signals(corr.size());
    calculateSmoothedZScore(corr, &signals);
}

void ZScoreEval::calculateSmoothedZScore(const std::vector<double>& input, 
    std::vector<int>* signals) const {
 //lag 5 for the smoothing functions
    size_t lag = 5;
    //3.5 standard deviations for signal
    double threshold = 3.5;
    //between 0 and 1, where 1 is normal influence, 0.5 is half
    double influence = .5;

    const uint32_t n_input = input.size();

    if (input.size() <= lag + 2)
        return;

    //Initialise variables
    std::vector<double> filteredY(input.size(), 0.0);
    std::vector<double> avgFilter(input.size(), 0.0);
    std::vector<double> stdFilter(input.size(), 0.0);
    std::vector<double> subVecStart(input.begin(), input.begin() + lag);
    avgFilter[lag] = mean(subVecStart);
    stdFilter[lag] = stdDev(subVecStart);

    for (size_t i = lag + 1; i < n_input; ++i)
    {
        if (std::abs(input[i] - avgFilter[i - 1]) > threshold * stdFilter[i - 1])
        {
            if (input[i] > avgFilter[i - 1])
            {
                (*signals)[i] = 1; 
            }
            else
            {
                (*signals)[i] = -1;
            }
            //Make influence lower
            filteredY[i] = influence* input[i] + (1 - influence) * filteredY[i - 1];
        }
        else
        {
            (*signals)[i] = 0; //# No signal
            filteredY[i] = input[i];
        }
        //Adjust the filters
        std::vector<double> subVec(filteredY.begin() + i - lag, filteredY.begin() + i);
        avgFilter[i] = mean(subVec);
        stdFilter[i] = stdDev(subVec);
    }
    //return signals;
}

double ZScoreEval::mean(const std::vector<double>& vec) const {
  return 0.0;
}

double ZScoreEval::stdDev(const std::vector<double>& vec) const {
  return 0.0;
}

}
