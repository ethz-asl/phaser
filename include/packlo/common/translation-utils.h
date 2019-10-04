#pragma once

#include "packlo/model/point-cloud.h"
#include "packlo/model/function-value.h"

#include <Eigen/Dense>
#include <vector>

namespace common {

class TranslationUtils {
public:
  static void TranslateXYZ(model::PointCloud &cloud, const float x, 
      const float y, const float z);
  static model::PointCloud TranslateXYZCopy(model::PointCloud &cloud,
      const float x, const float y, const float z);

private:
  static Eigen::Matrix4f createTransformationXYZ(
			const float x, const float y, const float z);

};

} // namespace common
