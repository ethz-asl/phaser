#include "packlo/backend/registration/mock/sph-registration-mock-rotated.h"
#include "packlo/common/rotation-utils.h"
#include "packlo/visualization/debug-visualizer.h"

#include <glog/logging.h>

DEFINE_double(
    mock_rotate_alpha_rad, M_PI / 2.0f,
    "Defines a mock rotation around roll in rad");
DEFINE_double(
    mock_rotate_beta_rad, M_PI / 2.2f,
    "Defines a mock rotation around pitch in rad");
DEFINE_double(
    mock_rotate_gamma_rad, M_PI / 2.5f,
    "Defines a mock rotation around yaw in rad");

namespace registration {

SphRegistrationMockRotated::SphRegistrationMockRotated()
    : mock_alpha_rad_(FLAGS_mock_rotate_alpha_rad),
      mock_beta_rad_(FLAGS_mock_rotate_beta_rad),
      mock_gamma_rad_(FLAGS_mock_rotate_gamma_rad) {}

model::RegistrationResult SphRegistrationMockRotated::registerPointCloud(
    model::PointCloudPtr cloud_prev, model::PointCloudPtr) {
  cloud_prev->initialize_kd_tree();

  model::PointCloud syn_cloud = pertubPointCloud(
      *cloud_prev, mock_alpha_rad_, mock_beta_rad_, mock_gamma_rad_);
  syn_cloud.initialize_kd_tree();

  std::array<double, 3> zyz;
  correlatePointcloud(*cloud_prev, syn_cloud, &zyz);
  Eigen::VectorXd xyz = common::RotationUtils::ConvertZYZtoXYZ(zyz);
  VLOG(1) << "estimated xyz: " << xyz.transpose();

  common::RotationUtils::RotateAroundXYZ(&syn_cloud, xyz(0), xyz(1), xyz(2));

  // visualization::DebugVisualizer::getInstance()
  // .visualizePointCloudDiff(*cloud_prev, syn_cloud);
  return model::RegistrationResult(std::move(syn_cloud), std::move(zyz));
}

void SphRegistrationMockRotated::setRandomRotation(
    const double alpha_rad, const double beta_rad, const double gamma_rad) {
  mock_alpha_rad_ = alpha_rad;
  mock_beta_rad_ = beta_rad;
  mock_gamma_rad_ = gamma_rad;
}

model::PointCloud SphRegistrationMockRotated::pertubPointCloud(
    const model::PointCloud& cloud, const float alpha_rad, const float beta_rad,
    const float gamma_rad) {
  VLOG(2) << "Pertubing point cloud using " << alpha_rad << ", " << beta_rad
          << ", " << gamma_rad << " XYZ angles.";
  return common::RotationUtils::RotateAroundZYXCopy(
      cloud, alpha_rad, beta_rad, gamma_rad);
}

}  // namespace registration
