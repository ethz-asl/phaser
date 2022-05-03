#ifndef PHASER_COMMON_TRANSLATION_UTILS_H_
#define PHASER_COMMON_TRANSLATION_UTILS_H_

#include <Eigen/Dense>
#include <vector>

#include "phaser/model/function-value.h"
#include "phaser/model/point-cloud.h"

namespace common {

class TranslationUtils {
 public:
  static void TranslateXYZ(
      model::PointCloud* cloud, const float x, const float y, const float z);
  static void TranslateXYZ(
      model::PointCloudPtr cloud, const float x, const float y, const float z);
  static model::PointCloud TranslateXYZCopy(
      const model::PointCloud& cloud, const float x, const float y,
      const float z);

  static double ComputeTranslationFromIndex(
      const double index, const uint32_t n_voxels,
      const int discretize_lower_bound, const int discretize_upper_bound);

  static std::array<uint32_t, 3> Ind2sub(
      const uint32_t lin_index, const uint32_t n_voxels);

  static std::array<uint32_t, 3> Ind2sub(
      const uint32_t lin_index, const uint32_t rows, const uint32_t cols);

 private:
  static Eigen::Matrix4f createTransformationXYZ(
      const float x, const float y, const float z);
};

}  // namespace common

#endif  // PHASER_COMMON_TRANSLATION_UTILS_H_
