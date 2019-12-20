#include <packlo/common/translation-utils.h>
#include <cmath>

#include <glog/logging.h>

namespace common {

void TranslationUtils::TranslateXYZ(
    model::PointCloud &cloud, const float x, const float y, const float z)  {
  Eigen::Matrix4f T = createTransformationXYZ(x, y, z);
  cloud.transformPointCloud(T);
}

model::PointCloud TranslationUtils::TranslateXYZCopy(
    const model::PointCloud &cloud,
    const float x, const float y, const float z) {
  Eigen::Matrix4f T = createTransformationXYZ(x, y, z);
  common::PointCloud_tPtr copyCloud(new common::PointCloud_t);
  model::PointCloud copy(copyCloud);
  cloud.transformPointCloudCopy(T, &copy);
  return copy;
}

Eigen::Matrix4f TranslationUtils::createTransformationXYZ(
    const float x, const float y, const float z) {
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0, 3) = x;
  T(1, 3) = y;
  T(2, 3) = z;

  return T;
}

}  // namespace common
