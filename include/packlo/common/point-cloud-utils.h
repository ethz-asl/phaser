#pragma once

#include "packlo/model/point-cloud.h"

namespace common {

class PointCloudUtils {
  public:
    void performZeroMeaning(model::PointCloud& cloud);
};

} // namespace common
