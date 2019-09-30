#include "packlo/backend/registration/mock/sph-registration-mock-rotated.h"
#include "packlo/common/rotation-utils.h"
#include "packlo/visualization/debug-visualizer.h"

#include <glog/logging.h>

DEFINE_double(mock_rotate_alpha_rad, M_PI/2.0f, 
		"Defines a mock rotation around roll in rad");
DEFINE_double(mock_rotate_beta_rad, M_PI/2.2f, 
		"Defines a mock rotation around pitch in rad");
DEFINE_double(mock_rotate_gamma_rad, M_PI/2.5f, 
		"Defines a mock rotation around yaw in rad");

namespace registration {

void SphRegistrationMockRotated::registerPointCloud(
		model::PointCloudPtr cloud_prev, 
		model::PointCloudPtr) {
  cloud_prev->initialize_kd_tree();

  model::PointCloud syn_cloud = pertubPointCloud(*cloud_prev, 
      FLAGS_mock_rotate_alpha_rad, 
			FLAGS_mock_rotate_beta_rad, 
			FLAGS_mock_rotate_gamma_rad);
  syn_cloud.initialize_kd_tree();

	visualization::DebugVisualizer::getInstance()
		.visualizePointCloudDiff(*cloud_prev, syn_cloud);  

	std::array<double, 3> zyz;
	correlatePointcloud(*cloud_prev, syn_cloud, &zyz);
  
  model::PointCloud reg_cloud = common::RotationUtils::RotateAroundZYZCopy(
      syn_cloud, zyz[2], zyz[1], zyz[0]);
	visualization::DebugVisualizer::getInstance()
		.visualizePointCloudDiff(*cloud_prev, reg_cloud);  
}

model::PointCloud SphRegistrationMockRotated::pertubPointCloud(
		model::PointCloud &cloud, 
    const float alpha_rad, const float beta_rad, const float gamma_rad) {
  return common::RotationUtils::RotateAroundXYZCopy(
      cloud, alpha_rad, beta_rad, gamma_rad);
}

} // namespace handler
