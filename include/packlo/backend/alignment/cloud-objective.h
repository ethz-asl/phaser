#pragma once

#include "packlo/backend/alignment/base-objective.h"

#include <vector>

namespace alignment {
  
class CloudObjective : public BaseObjective {
  public:
    virtual double optimize(const std::vector<double>& x) override;
    virtual void calculateGrad(std::vector<double>& grad) override;
};
  
} // namespace alignment
