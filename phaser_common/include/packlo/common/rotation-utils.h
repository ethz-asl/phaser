#ifndef PACKLO_COMMON_ROTATION_UTILS_H_
#define PACKLO_COMMON_ROTATION_UTILS_H_

#include "packlo/model/point-cloud.h"
#include "packlo/model/function-value.h"

#include <Eigen/Dense>
#include <vector>
#include <array>

namespace common {

class RotationUtils {
 public:
  static void RotateAroundZYX(
      model::PointCloud* cloud, const float alpha_rad, const float beta_rad,
      const float gamma_rad);
  static void RotateAroundZYX(
      model::PointCloudPtr cloud, const float alpha_rad, const float beta_rad,
      const float gamma_rad);
  static model::PointCloud RotateAroundZYXCopy(
      const model::PointCloud& cloud, const float alpha_rad,
      const float beta_rad, const float gamma_rad);
  static void RotateAroundXYZ(
      model::PointCloud* cloud, const float alpha_rad, const float beta_rad,
      const float gamma_rad);
  static void RotateAroundXYZ(
      model::PointCloudPtr cloud, const float alpha_rad, const float beta_rad,
      const float gamma_rad);
  static void RotateAroundXYZInv(
      model::PointCloudPtr cloud, const float alpha_rad, const float beta_rad,
      const float gamma_rad);
  static model::PointCloud RotateAroundXYZCopy(
      const model::PointCloud& cloud, const float alpha_rad,
      const float beta_rad, const float gamma_rad);

  static void RotateAroundZYZ(
      model::PointCloud* cloud, const double alpha_rad, const double beta_rad,
      const double gamma_rad);
  static model::PointCloud RotateAroundZYZCopy(
      const model::PointCloud& cloud, const double alpha_rad,
      const double beta_rad, const double gamma_rad);

  static std::vector<model::FunctionValue> RotateAroundZYZCopy(
      const std::vector<model::FunctionValue>& values, const double alpha_rad,
      const double beta_rad, const double gamma_rad);

  static Eigen::Vector3d ConvertZYZtoXYZ(const std::array<double, 3>& zyz);
  static Eigen::Quaterniond ConvertZYZtoQuaternion(
      const std::array<double, 3>& zyz);
  static Eigen::Vector3d ConvertQuaternionToXYZ(const Eigen::Quaterniond& q);
  static Eigen::Vector3d ConvertQuaternionToXYZ(const Eigen::Vector4d& q);

  static Eigen::MatrixXd ConvertQuaternionsToMatrix(
      const std::vector<Eigen::Quaterniond>& quaternions);

 private:
  static Eigen::Matrix4f createTransformationAroundX(const float alpha_rad);
  static Eigen::Matrix4f createTransformationAroundY(const float beta_rad);
  static Eigen::Matrix4f createTransformationAroundZ(const float gamma_rad);

  static Eigen::Vector3d fromRotation(const double r11, const double r12,
      const double r21, const double r31, const double r32);
};

}  // namespace common

#endif  // PACKLO_COMMON_ROTATION_UTILS_H_
