#pragma once

#include <packlo/model/point-cloud.h>

#include <Eigen/Dense>

namespace common {

class RotationUtils {
public:
  static void RotateAroundXYZ(model::PointCloud &cloud, const float alpha_rad, 
      const float beta_rad, const float gamma_rad);
  static model::PointCloud RotateAroundXYZCopy(model::PointCloud &cloud,
      const float alpha_rad, const float beta_rad, const float gamma_rad);

private:
  static Eigen::Matrix4f createTransformationAroundX(const float alpha_rad);
  static Eigen::Matrix4f createTransformationAroundY(const float beta_rad);
  static Eigen::Matrix4f createTransformationAroundZ(const float gamma_rad);

};

}
