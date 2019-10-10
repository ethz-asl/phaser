#pragma once

#include <vector>

namespace alignment {
  
class Objective {
  public:
    double optimize(const std::vector<double>& x, 
        std::vector<double>& grad);
    void calculateGrad(std::vector<double>& grad);
};
  
} // namespace alignment
