#pragma once

#include "packlo/model/point-cloud.h"
#include "packlo/model/function-value.h"

#include <Eigen/Dense>
#include <vector>

namespace common {

class RotationUtils {
public:
  static void RotateAroundXYZ(model::PointCloud &cloud, const float alpha_rad, 
      const float beta_rad, const float gamma_rad);
  static model::PointCloud RotateAroundXYZCopy(model::PointCloud &cloud,
      const float alpha_rad, const float beta_rad, const float gamma_rad);

  static void RotateAroundZYZ(model::PointCloud &cloud, const double alpha_rad, 
      const double beta_rad, const double gamma_rad);
  static model::PointCloud RotateAroundZYZCopy(model::PointCloud &cloud,
      const double alpha_rad, const double beta_rad, const double gamma_rad);

  static std::vector<model::FunctionValue> RotateAroundZYZCopy(
			const std::vector<model::FunctionValue>& values,
      const double alpha_rad, const double beta_rad, const double gamma_rad);

private:
  static Eigen::Matrix4f createTransformationAroundX(const float alpha_rad);
  static Eigen::Matrix4f createTransformationAroundY(const float beta_rad);
  static Eigen::Matrix4f createTransformationAroundZ(const float gamma_rad);

};

} // namespace common
