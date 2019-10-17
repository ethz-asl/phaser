#pragma once

#include "packlo/backend/alignment/base-objective.h"

#include <vector>

namespace alignment {
  
class FunctionObjective : public BaseObjective {
  public:
    virtual double optimize(const std::vector<double>& x) override;
    virtual void calculateGrad(std::vector<double>& grad) override;

  private:
    void convertFunctionValuesToPointMatrix(
        const std::vector<model::FunctionValue>& values, 
        Eigen::MatrixXf& matrix);
    void convertFunctionValuesToPointcloud(
        const std::vector<model::FunctionValue>& values, 
        model::PointCloud& cloud);
};
  
} // namespace alignment
