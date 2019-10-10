#include "packlo/backend/alignment/objective.h"

namespace alignment {

double Objective::optimize(const std::vector<double>& x, 
    std::vector<double>& grad) {
  return 0.0;
}

void Objective::calculateGrad(std::vector<double>& grad) {

}

} // namespace alignment
