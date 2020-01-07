#include "packlo/common/rotation-utils.h"

#include <glog/logging.h>

#include <cmath>

namespace common {

void RotationUtils::RotateAroundXYZ(
    model::PointCloud* cloud, const float alpha_rad, const float beta_rad,
    const float gamma_rad) {
  Eigen::Matrix4f T = createTransformationAroundZ(gamma_rad) *
                      createTransformationAroundY(beta_rad) *
                      createTransformationAroundX(alpha_rad);
  cloud->transformPointCloud(T);
}

model::PointCloud RotationUtils::RotateAroundXYZCopy(
    const model::PointCloud& cloud, const float alpha_rad, const float beta_rad,
    const float gamma_rad) {
  Eigen::Matrix4f T = createTransformationAroundZ(gamma_rad) *
                      createTransformationAroundY(beta_rad) *
                      createTransformationAroundX(alpha_rad);
  common::PointCloud_tPtr copyCloud(new common::PointCloud_t);
  model::PointCloud copy(copyCloud);
  cloud.transformPointCloudCopy(T, &copy);
  return copy;
}

std::vector<model::FunctionValue> RotationUtils::RotateAroundZYZCopy(
    const std::vector<model::FunctionValue>& values,
    const double alpha_rad, const double beta_rad, const double gamma_rad) {
  Eigen::Matrix4f T = createTransformationAroundZ(gamma_rad) *
                      createTransformationAroundY(beta_rad) *
                      createTransformationAroundX(alpha_rad);
  std::vector<model::FunctionValue> rotated_values;
  // TODO(lbern:) rotate or remove
  return rotated_values;
}

Eigen::Vector3d RotationUtils::ConvertZYZtoXYZ(
    const std::array<double, 3>& zyz) {
  Eigen::Quaterniond q = Eigen::AngleAxisd(zyz[0], Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(zyz[1], Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(zyz[2], Eigen::Vector3d::UnitZ());
  const double qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
  return fromRotation(
      -2 * (qy * qz - qw * qx), qw * qw - qx * qx - qy * qy + qz * qz,
      2 * (qx * qz + qw * qy), -2 * (qx * qy - qw * qz),
      qw * qw + qx * qx - qy * qy - qz * qz);
}

Eigen::Matrix4f RotationUtils::createTransformationAroundX(
    const float alpha_rad) {
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(1, 1) = std::cos(alpha_rad);
  T(1, 2) = -std::sin(alpha_rad);
  T(2, 1) = std::sin(alpha_rad);
  T(2, 2) = std::cos(alpha_rad);

  return T;
}

Eigen::Matrix4f RotationUtils::createTransformationAroundY(
    const float beta_rad) {
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0, 0) = std::cos(beta_rad);
  T(0, 2) = std::sin(beta_rad);
  T(2, 0) = -std::sin(beta_rad);
  T(2, 2) = std::cos(beta_rad);

  return T;
}

Eigen::Matrix4f RotationUtils::createTransformationAroundZ(
    const float gamma_rad) {
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0, 0) = std::cos(gamma_rad);
  T(0, 1) = -std::sin(gamma_rad);
  T(1, 0) = std::sin(gamma_rad);
  T(1, 1) = std::cos(gamma_rad);

  return T;
}

void RotationUtils::RotateAroundZYZ(
    model::PointCloud* cloud, const double alpha_rad, const double beta_rad,
    const double gamma_rad) {
  Eigen::Matrix4f T = createTransformationAroundZ(gamma_rad) *
                      createTransformationAroundY(beta_rad) *
                      createTransformationAroundZ(alpha_rad);
  cloud->transformPointCloud(T);
}

model::PointCloud RotationUtils::RotateAroundZYZCopy(
    const model::PointCloud &cloud,
    const double alpha_rad, const double beta_rad, const double gamma_rad) {
  Eigen::Matrix4f T = createTransformationAroundZ(gamma_rad) *
                      createTransformationAroundY(beta_rad) *
                      createTransformationAroundZ(alpha_rad);
  common::PointCloud_tPtr copyCloud(new common::PointCloud_t);
  model::PointCloud copy(copyCloud);
  cloud.transformPointCloudCopy(T, &copy);
  return copy;
}

Eigen::Vector3d RotationUtils::fromRotation(const double r11, const double r12,
    const double r21, const double r31, const double r32) {
  Eigen::Vector3d res;
  res(0) = std::atan2(r31, r32);
  res(1) = std::asin(r21);
  res(2) = std::atan2(r11, r12);
  return res;
}
Eigen::MatrixXd RotationUtils::ConvertQuaternionsToMatrix(
    const std::vector<Eigen::Quaterniond>& quaternions) {
  const uint16_t n_quaternions = quaternions.size();
  Eigen::MatrixXd samples = Eigen::MatrixXd::Zero(4, n_quaternions);
  for (uint16_t i = 0u; i < n_quaternions; ++i) {
    const Eigen::Quaterniond& q = quaternions[i];
    samples(0, i) = q.w();
    samples(1, i) = q.x();
    samples(2, i) = q.y();
    samples(3, i) = q.z();
  }

  return samples;
}

}  // namespace common
