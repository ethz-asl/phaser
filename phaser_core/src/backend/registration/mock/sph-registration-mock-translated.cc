#include "packlo/backend/registration/mock/sph-registration-mock-translated.h"
#include "packlo/common/translation-utils.h"
#include "packlo/common/rotation-utils.h"
#include "packlo/common/statistic-utils.h"

#include <glog/logging.h>

DEFINE_double(mock_translate_x, 5, "Defines a mock translation in x.");
DEFINE_double(mock_translate_y, 5, "Defines a mock translation in y.");
DEFINE_double(mock_translate_z, 5, "Defines a mock translation in z.");

namespace registration {

SphRegistrationMockTranslated::SphRegistrationMockTranslated()
    : mock_trans_x_(FLAGS_mock_translate_x),
      mock_trans_y_(FLAGS_mock_translate_y),
      mock_trans_z_(FLAGS_mock_translate_z) {}

model::RegistrationResult SphRegistrationMockTranslated::registerPointCloud(
    model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur) {
  cloud_prev->initialize_kd_tree();

  model::PointCloud syn_cloud = pertubPointCloud(
      *cloud_prev, mock_trans_x_, mock_trans_y_, mock_trans_z_);
  /*
  model::PointCloud& syn_cloud = *cloud_cur;
  */
  syn_cloud.initialize_kd_tree();

  sampler_.sampleUniformly(*cloud_prev, &f_values_);
  sampler_.sampleUniformly(syn_cloud, &h_values_);
  std::vector<model::FunctionValue> syn_values;
  /*
    pertubFunctionValues(f_values_, FLAGS_mock_translate_x, 
        FLAGS_mock_translate_y, FLAGS_mock_translate_z);
        */

  common::Vector_t xyz;
  const double duration_translation_f_ms = common::executeTimedFunction(
      &alignment::BaseAligner::alignRegistered, &(*aligner_), *cloud_prev,
      f_values_, syn_cloud, syn_values, &xyz);

  CHECK_EQ(xyz.rows(), 3);
  VLOG(1) << "Found translation: " << xyz.transpose();
  VLOG(1) << "Translational alignment took: " << duration_translation_f_ms
    << "ms.";
  model::PointCloud reg_cloud = common::TranslationUtils::TranslateXYZCopy(
      syn_cloud, xyz(0), xyz(1), xyz(2));

  /*
  const std::vector<double> corr = aligner_->getCorrelation();
  eval_->evaluateCorrelationFromTranslation(corr);
  */

  return model::RegistrationResult(std::move(reg_cloud), std::move(xyz));
}

void SphRegistrationMockTranslated::setRandomTranslation(
    const double mock_trans_x, const double mock_trans_y,
    const double mock_trans_z) {
  mock_trans_x_ = mock_trans_x;
  mock_trans_y_ = mock_trans_y;
  mock_trans_z_ = mock_trans_z;
}

model::PointCloud SphRegistrationMockTranslated::pertubPointCloud(
    model::PointCloud &cloud,
    const float x, const float y, const float z) {
  return common::TranslationUtils::TranslateXYZCopy(cloud, x, y, z);
}

std::vector<model::FunctionValue>
SphRegistrationMockTranslated::pertubFunctionValues(
    std::vector<model::FunctionValue>& values, const float x, const float y,
    const float z) {
  std::vector<model::FunctionValue> res;
  for (model::FunctionValue& val : values) {
    model::FunctionValue cur;
    common::Point_t cur_point = val.getAveragedPoint();
    cur_point.x += x;
    cur_point.y += y;
    cur_point.z += z;
    double range = std::sqrt(
        cur_point.x * cur_point.x + cur_point.y * cur_point.y +
        cur_point.z * cur_point.z);
    cur.addRange(range);
    cur.addPoint(cur_point);
    res.emplace_back(cur);
  }
  return res;
}

}  // namespace registration
