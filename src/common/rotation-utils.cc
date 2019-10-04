#include <packlo/common/rotation-utils.h>
#include <cmath>

#include <glog/logging.h>

namespace common {

void RotationUtils::RotateAroundXYZ(model::PointCloud &cloud, const float alpha_rad, 
    const float beta_rad, const float gamma_rad) {
  Eigen::Matrix4f T = createTransformationAroundZ(gamma_rad) *
    createTransformationAroundY(beta_rad) * createTransformationAroundX(alpha_rad);
  cloud.transformPointCloud(T);
}

model::PointCloud RotationUtils::RotateAroundXYZCopy(model::PointCloud &cloud, 
    const float alpha_rad, 
    const float beta_rad, const float gamma_rad) {
  Eigen::Matrix4f T = createTransformationAroundZ(gamma_rad) *
                      createTransformationAroundY(beta_rad) * 
                      createTransformationAroundX(alpha_rad);
  common::PointCloud_tPtr copyCloud (new common::PointCloud_t);
  model::PointCloud copy (copyCloud);
  cloud.transformPointCloudCopy(T, copy);
  return copy;
}

std::vector<model::FunctionValue> RotationUtils::RotateAroundZYZCopy(
    const std::vector<model::FunctionValue>& values,
    const double alpha_rad, const double beta_rad, const double gamma_rad) {
  Eigen::Matrix4f T = createTransformationAroundZ(gamma_rad) *
                      createTransformationAroundY(beta_rad) * 
                      createTransformationAroundX(alpha_rad);
  std::vector<model::FunctionValue> rotated_values;
  //TODO(lbern:) rotate or remove
  return rotated_values;
}

Eigen::Matrix4f RotationUtils::createTransformationAroundX(const float alpha_rad) {
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity(); 
  T(1,1) = std::cos (alpha_rad);
  T(1,2) = -std::sin(alpha_rad);
  T(2,1) = std::sin (alpha_rad);
  T(2,2) = std::cos (alpha_rad);

  return T;
}

Eigen::Matrix4f RotationUtils::createTransformationAroundY(const float beta_rad) {
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity(); 
  T(0,0) = std::cos (beta_rad);
  T(0,2) = std::sin(beta_rad);
  T(2,0) = -std::sin (beta_rad);
  T(2,2) = std::cos (beta_rad);

  return T;
}

Eigen::Matrix4f RotationUtils::createTransformationAroundZ(const float gamma_rad) {
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity(); 
  T(0,0) = std::cos (gamma_rad);
  T(0,1) = -std::sin(gamma_rad);
  T(1,0) = std::sin (gamma_rad);
  T(1,1) = std::cos (gamma_rad);

  return T;
}

void RotationUtils::RotateAroundZYZ(model::PointCloud &cloud, const double alpha_rad, 
    const double beta_rad, const double gamma_rad) {
  Eigen::Matrix4f T = createTransformationAroundZ(gamma_rad) *
    createTransformationAroundY(beta_rad) * createTransformationAroundZ(alpha_rad);
  cloud.transformPointCloud(T);
}

model::PointCloud RotationUtils::RotateAroundZYZCopy(model::PointCloud &cloud,
    const double alpha_rad, const double beta_rad, const double gamma_rad) {
  Eigen::Matrix4f T = createTransformationAroundZ(gamma_rad) *
   createTransformationAroundY(beta_rad) * createTransformationAroundZ(alpha_rad);
  common::PointCloud_tPtr copyCloud (new common::PointCloud_t);
  model::PointCloud copy (copyCloud);
  cloud.transformPointCloudCopy(T, copy);
  return copy;
}

}
