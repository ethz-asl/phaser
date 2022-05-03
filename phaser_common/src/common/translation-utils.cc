#include "phaser/common/translation-utils.h"

#include <cmath>
#include <glog/logging.h>

namespace common {

void TranslationUtils::TranslateXYZ(
    model::PointCloudPtr cloud, const float x, const float y, const float z) {
  Eigen::Matrix4f T = createTransformationXYZ(x, y, z);
  cloud->transformPointCloud(T);
}

void TranslationUtils::TranslateXYZ(
    model::PointCloud* cloud, const float x, const float y, const float z) {
  Eigen::Matrix4f T = createTransformationXYZ(x, y, z);
  cloud->transformPointCloud(T);
}

model::PointCloud TranslationUtils::TranslateXYZCopy(
    const model::PointCloud& cloud, const float x, const float y,
    const float z) {
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

double TranslationUtils::ComputeTranslationFromIndex(
    const double index, const uint32_t n_voxels,
    const int discretize_lower_bound, const int discretize_upper_bound) {
  if (index == 0)
    return 0.0;
  const double n_voxels_half = n_voxels / 2.0;
  const double width =
      std::abs(discretize_lower_bound) + std::abs(discretize_upper_bound) + 1;
  if ((index) <= n_voxels_half) {
    return (index)*width / n_voxels;
  }
  return (index - n_voxels) * width / n_voxels;
}

std::array<uint32_t, 3> TranslationUtils::Ind2sub(
    const uint32_t lin_index, const uint32_t n_voxels) {
  return Ind2sub(lin_index, n_voxels, n_voxels);
}

std::array<uint32_t, 3> TranslationUtils::Ind2sub(
    const uint32_t lin_index, const uint32_t rows, const uint32_t cols) {
  std::array<uint32_t, 3> xyz;
  xyz[1] = lin_index % cols;
  const int updated_index = lin_index / cols;
  xyz[0] = updated_index % rows;
  xyz[2] = updated_index / rows;
  return xyz;
}

}  // namespace common
