#include "phaser/common/rotation-utils.h"

#include <cmath>
#include <glog/logging.h>

namespace common {

void RotationUtils::RotateAroundXYZ(
    model::PointCloud* cloud, const float alpha_rad, const float beta_rad,
    const float gamma_rad) {
  Eigen::Matrix4f T = createTransformationAroundX(alpha_rad) *
                      createTransformationAroundY(beta_rad) *
                      createTransformationAroundZ(gamma_rad);
  cloud->transformPointCloud(T);
}

void RotationUtils::RotateAroundXYZ(
    model::PointCloudPtr cloud, const float alpha_rad, const float beta_rad,
    const float gamma_rad) {
  Eigen::Matrix4f T = createTransformationAroundX(alpha_rad) *
                      createTransformationAroundY(beta_rad) *
                      createTransformationAroundZ(gamma_rad);
  cloud->transformPointCloud(T);
}

void RotationUtils::RotateAroundXYZInv(
    model::PointCloudPtr cloud, const float alpha_rad, const float beta_rad,
    const float gamma_rad) {
  Eigen::Matrix4f T = createTransformationAroundX(alpha_rad) *
                      createTransformationAroundY(beta_rad) *
                      createTransformationAroundZ(gamma_rad);
  cloud->transformPointCloud(T.inverse());
}

void RotationUtils::RotateAroundZYX(
    model::PointCloudPtr cloud, const float alpha_rad, const float beta_rad,
    const float gamma_rad) {
  Eigen::Matrix4f T = createTransformationAroundZ(gamma_rad) *
                      createTransformationAroundY(beta_rad) *
                      createTransformationAroundX(alpha_rad);
  cloud->transformPointCloud(T);
}

void RotationUtils::RotateAroundZYX(
    model::PointCloud* cloud, const float alpha_rad, const float beta_rad,
    const float gamma_rad) {
  Eigen::Matrix4f T = createTransformationAroundZ(gamma_rad) *
                      createTransformationAroundY(beta_rad) *
                      createTransformationAroundX(alpha_rad);
  cloud->transformPointCloud(T);
}

model::PointCloud RotationUtils::RotateAroundZYXCopy(
    const model::PointCloud& cloud, const float alpha_rad, const float beta_rad,
    const float gamma_rad) {
  Eigen::Matrix4f T = createTransformationAroundZ(gamma_rad) *
                      createTransformationAroundY(beta_rad) *
                      createTransformationAroundX(alpha_rad);
  VLOG(4) << "Transformation: \n" << T;
  common::PointCloud_tPtr copyCloud(new common::PointCloud_t);
  model::PointCloud copy(copyCloud);
  cloud.transformPointCloudCopy(T, &copy);
  return copy;
}

model::PointCloud RotationUtils::RotateAroundXYZCopy(
    const model::PointCloud& cloud, const float alpha_rad, const float beta_rad,
    const float gamma_rad) {
  Eigen::Matrix4f T = createTransformationAroundX(alpha_rad) *
                      createTransformationAroundY(beta_rad) *
                      createTransformationAroundZ(gamma_rad);
  VLOG(4) << "Transformation: \n" << T;

  common::PointCloud_tPtr copyCloud(new common::PointCloud_t);
  model::PointCloud copy(copyCloud);
  cloud.transformPointCloudCopy(T, &copy);
  return copy;
}

Eigen::Vector3d RotationUtils::ConvertZYZtoXYZ(
    const std::array<double, 3>& zyz) {
  Eigen::Quaterniond q = ConvertZYZtoQuaternion(zyz);
  return ConvertQuaternionToXYZ(q);
}

Eigen::Vector3d RotationUtils::ConvertQuaternionToXYZ(
    const Eigen::Quaterniond& q) {
  const double qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
  return fromRotation(
      -2 * (qy * qz - qw * qx), qw * qw - qx * qx - qy * qy + qz * qz,
      2 * (qx * qz + qw * qy), -2 * (qx * qy - qw * qz),
      qw * qw + qx * qx - qy * qy - qz * qz);
}

Eigen::Vector3d RotationUtils::ConvertQuaternionToXYZ(
    const Eigen::Vector4d& q) {
  Eigen::Quaterniond e_q(q(0), q(1), q(2), q(3));
  return ConvertQuaternionToXYZ(e_q);
}

Eigen::Quaterniond RotationUtils::ConvertZYZtoQuaternion(
    const std::array<double, 3>& zyz) {
  Eigen::Quaterniond q = Eigen::AngleAxisd(zyz[0], Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(zyz[1], Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(zyz[2], Eigen::Vector3d::UnitZ());
  return q;
}

Eigen::Matrix4f RotationUtils::createTransformationAroundX(
    const float alpha_rad) {
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(1, 1) = std::cos(alpha_rad);
  T(1, 2) = -std::sin(alpha_rad);
  T(2, 1) = std::sin(alpha_rad);
  T(2, 2) = std::cos(alpha_rad);
  VLOG(4) << "Tx (" << alpha_rad << "rad) = \n" << T;

  return T;
}

Eigen::Matrix4f RotationUtils::createTransformationAroundY(
    const float beta_rad) {
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0, 0) = std::cos(beta_rad);
  T(0, 2) = std::sin(beta_rad);
  T(2, 0) = -std::sin(beta_rad);
  T(2, 2) = std::cos(beta_rad);
  VLOG(4) << "Ty (" << beta_rad << "rad) = \n" << T;

  return T;
}

Eigen::Matrix4f RotationUtils::createTransformationAroundZ(
    const float gamma_rad) {
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0, 0) = std::cos(gamma_rad);
  T(0, 1) = std::sin(gamma_rad);
  T(1, 0) = -std::sin(gamma_rad);
  T(1, 1) = std::cos(gamma_rad);
  VLOG(4) << "Tz (" << gamma_rad << "rad) = \n" << T;

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
    const model::PointCloud& cloud, const double alpha_rad,
    const double beta_rad, const double gamma_rad) {
  Eigen::Matrix4f T = createTransformationAroundZ(gamma_rad) *
                      createTransformationAroundY(beta_rad) *
                      createTransformationAroundZ(alpha_rad);
  common::PointCloud_tPtr copyCloud(new common::PointCloud_t);
  model::PointCloud copy(copyCloud);
  cloud.transformPointCloudCopy(T, &copy);
  return copy;
}

Eigen::Vector3d RotationUtils::fromRotation(
    const double r11, const double r12, const double r21, const double r31,
    const double r32) {
  Eigen::Vector3d res;
  res(2) = -std::atan2(r31, r32);
  res(1) = std::asin(r21);
  res(0) = std::atan2(r11, r12);
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

std::array<double, 3> RotationUtils::GetZYZFromIndex(
    const uint32_t index, const uint32_t bw) {
  std::array<double, 3> zyz;
  convertSO3toZYZ(index, bw, &zyz);
  return zyz;
}

void RotationUtils::convertSO3toZYZ(
    const uint32_t loc, const uint32_t bw, std::array<double, 3>* const zyz) {
  const int32_t ii = floor(loc / (4. * bw * bw));
  int32_t tmp = loc - (ii * 4. * bw * bw);
  const int32_t jj = floor(tmp / (2. * bw));
  const int32_t kk = loc - (ii * 4 * bw * bw) - jj * (2 * bw);

  const double bandwith = static_cast<double>(bw);
  (*zyz)[0] = M_PI * jj / (bandwith);
  (*zyz)[1] = M_PI * (2 * ii + 1) / (4. * bw);
  (*zyz)[2] = M_PI * kk / (bandwith);
}

}  // namespace common
