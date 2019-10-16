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

  /*
  visualization::DebugVisualizer::getInstance()
    .visualizePointCloudDiff(*cloud_prev, syn_cloud);  
    */

  sampler_.sampleUniformly(*cloud_prev, &f_values_);
  sampler_.sampleUniformly(syn_cloud, &h_values_);
  std::vector<model::FunctionValue> syn_values = 
    pertubFunctionValues(f_values_, FLAGS_mock_translate_x, 
        FLAGS_mock_translate_y, FLAGS_mock_translate_z);

  common::Vector_t xyz = aligner_->alignRegistered(
      *cloud_prev, f_values_, syn_cloud, syn_values);

  CHECK(xyz.rows() == 3);
  VLOG(1) << "estimated trans: " << xyz(0) << ", " << xyz(1) << ", " << xyz(2);
  model::PointCloud reg_cloud = common::TranslationUtils::TranslateXYZCopy(
      syn_cloud, xyz(0), xyz(1), xyz(2));
  /*
  visualization::DebugVisualizer::getInstance()
    .visualizePointCloudDiff(*cloud_prev, reg_cloud);  
    */
}

model::PointCloud SphRegistrationMockTranslated::pertubPointCloud(
    model::PointCloud &cloud,
    const float x, const float y, const float z) {
  return common::TranslationUtils::TranslateXYZCopy(cloud, x, y, z);
}

std::vector<model::FunctionValue> 
    SphRegistrationMockTranslated::pertubFunctionValues(
    std::vector<model::FunctionValue>& values,
    const float x, const float y, const float z) {
  std::vector<model::FunctionValue> res;
  for (model::FunctionValue& val : values) {
    model::FunctionValue cur;
    common::Point_t cur_point = val.getAveragedPoint();
    cur_point.x += x; 
    cur_point.y += y; 
    cur_point.z += z; 
    double range = std::sqrt(cur_point.x*cur_point.x + 
                             cur_point.y*cur_point.y +  
                             cur_point.z*cur_point.z);
    cur.addRange(range);
    cur.addPoint(cur_point);
    res.emplace_back(cur);
  }
  return res;
}

} // namespace handler
