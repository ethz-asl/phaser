#pragma once

#include "packlo/model/point-cloud.h"
#include "packlo/model/function-value.h"

#include <Eigen/Dense>
#include <vector>
#include <array>

namespace common {

class RotationUtils {
public:
  static void RotateAroundXYZ(model::PointCloud &cloud, const float alpha_rad, 
      const float beta_rad, const float gamma_rad);
  static model::PointCloud RotateAroundXYZCopy(const model::PointCloud &cloud,
      const float alpha_rad, const float beta_rad, const float gamma_rad);

  static void RotateAroundZYZ(model::PointCloud &cloud, const double alpha_rad, 
      const double beta_rad, const double gamma_rad);
  static model::PointCloud RotateAroundZYZCopy(const model::PointCloud &cloud,
      const double alpha_rad, const double beta_rad, const double gamma_rad);

  static std::vector<model::FunctionValue> RotateAroundZYZCopy(
      const std::vector<model::FunctionValue>& values,
      const double alpha_rad, const double beta_rad, const double gamma_rad);

  static Eigen::Vector3d ConvertZYZtoXYZ(const std::array<double, 3>& zyz);

private:
  static Eigen::Matrix4f createTransformationAroundX(const float alpha_rad);
  static Eigen::Matrix4f createTransformationAroundY(const float beta_rad);
  static Eigen::Matrix4f createTransformationAroundZ(const float gamma_rad);

  static Eigen::Vector3d fromRotation(const double r11, const double r12,
      const double r21, const double r31, const double r32);


};
 

} // namespace common