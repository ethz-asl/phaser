#include "packlo/common/point-cloud-utils.h"

namespace common {

void PointCloudUtils::performZeroMeaning(model::PointCloud& cloud) {
  Eigen::MatrixXf points = cloud.getRawCloud()->getMatrixXfMap();
  const Eigen::Vector3f mean = points.rowwise().mean();
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0,3) = mean(0);
  T(1,3) = mean(1);
  T(2,3) = mean(2);
  cloud.transformPointCloud(T);
}

} // namespace common
