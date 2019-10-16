#pragma once

#include "packlo/backend/alignment/base-aligner.h"
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>

namespace alignment {

class FeatureAligner : public BaseAligner {
  public:
    virtual void alignRegistered(
      const model::PointCloud& cloud_prev, 
      const std::vector<model::FunctionValue>& f_prev, 
      const model::PointCloud& cloud_reg,
      const std::vector<model::FunctionValue>& f_reg, 
      common::Vector_t* xyz) override;
  private: 
    void calculatePFH(const model::PointCloud& cloud, 
        pcl::PointCloud<pcl::PFHSignature125>::Ptr output);
    void calculateFPFH(const model::PointCloud& cloud,
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr output);
};

} // namespace alignment
