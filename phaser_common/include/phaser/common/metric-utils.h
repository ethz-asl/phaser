#pragma once

#include "phaser/model/point-cloud.h"

namespace common {

class MetricUtils {
 public:
  static float HausdorffDistance(
      const model::PointCloudPtr& cloud_a, const model::PointCloudPtr& cloud_b);
};

}  // namespace common
