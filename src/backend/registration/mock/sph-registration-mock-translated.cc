#include "packlo/backend/registration/mock/sph-registration-mock-translated.h"
#include "packlo/common/translation-utils.h"
#include "packlo/common/rotation-utils.h"
#include "packlo/visualization/debug-visualizer.h"

#include <glog/logging.h>

DEFINE_double(mock_translate_x, 5, 
		"Defines a mock translation in x.");
DEFINE_double(mock_translate_y, 5, 
		"Defines a mock translation in y.");
DEFINE_double(mock_translate_z, 5, 
		"Defines a mock translation in z.");

namespace registration {

void SphRegistrationMockTranslated::registerPointCloud(
		model::PointCloudPtr cloud_prev, 
		model::PointCloudPtr) {
  cloud_prev->initialize_kd_tree();

  model::PointCloud syn_cloud = pertubPointCloud(*cloud_prev, 
      FLAGS_mock_translate_x, 
			FLAGS_mock_translate_y, 
			FLAGS_mock_translate_z);
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

model::PointCloud SphRegistrationMockTranslated::pertubPointCloud(
		model::PointCloud &cloud,
		const float x, const float y, const float z) {
	return common::TranslationUtils::TranslateXYZCopy(cloud, x, y, z);
}

} // namespace handler
