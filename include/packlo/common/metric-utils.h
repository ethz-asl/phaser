#pragma once

#include "packlo/model/point-cloud.h"

namespace common {

class MetricUtils {
  public:
    float calcHausdorffDistance(const model::PointCloudPtr& cloud_a,                   
      const model::PointCloudPtr& cloud_b) const;
};

} // namespace common
