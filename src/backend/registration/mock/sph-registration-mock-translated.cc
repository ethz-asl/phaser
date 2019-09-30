#include "packlo/backend/registration/mock/sph-registration-mock-translated.h"
#include "packlo/common/rotation-utils.h"
#include "packlo/visualization/debug-visualizer.h"

#include <glog/logging.h>

namespace registration {

void SphRegistrationMockTranslated::registerPointCloud(
		model::PointCloudPtr cloud_prev, 
		model::PointCloudPtr) {
  cloud_prev->initialize_kd_tree();

	/*
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
		*/
}


} // namespace handler
