#pragma once

#include "phaser/model/point-cloud.h"

namespace common {

class PointCloudUtils {
 public:
  static model::PointCloud performZeroMeaning(const model::PointCloud& cloud);
};

}  // namespace common
