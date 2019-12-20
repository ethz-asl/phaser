#pragma once

#include "packlo/model/point-cloud.h"
#include "packlo/model/function-value.h"

#include <vector>

namespace alignment {
  
class BaseObjective {
  public:
    virtual double optimize(const std::vector<double>& x) = 0;
    virtual void calculateGrad(std::vector<double>& grad) = 0;

    void setPrevious(const model::PointCloud& cloud_prev, 
        const std::vector<model::FunctionValue>& f_prev);

    void setCurrent(const model::PointCloud& cloud_cur, 
        const std::vector<model::FunctionValue>& f_cur);

  protected:
    const model::PointCloud* cloud_prev_;
    const std::vector<model::FunctionValue>* f_prev_;

    const model::PointCloud* cloud_cur_;
    const std::vector<model::FunctionValue>* f_cur_;
};
  
} // namespace alignment
