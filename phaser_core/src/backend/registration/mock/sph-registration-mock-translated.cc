#include "phaser/backend/registration/mock/sph-registration-mock-translated.h"
#include "phaser/common/translation-utils.h"
#include "phaser/common/rotation-utils.h"
#include "phaser/common/statistic-utils.h"

#include <glog/logging.h>

DEFINE_double(mock_translate_x, 5, "Defines a mock translation in x.");
DEFINE_double(mock_translate_y, 5, "Defines a mock translation in y.");
DEFINE_double(mock_translate_z, 5, "Defines a mock translation in z.");

namespace phaser_core {

SphRegistrationMockTranslated::SphRegistrationMockTranslated()
    : mock_trans_x_(FLAGS_mock_translate_x),
      mock_trans_y_(FLAGS_mock_translate_y),
      mock_trans_z_(FLAGS_mock_translate_z) {}

model::RegistrationResult SphRegistrationMockTranslated::registerPointCloud(
    model::PointCloudPtr cloud_prev, model::PointCloudPtr) {
  cloud_prev->initialize_kd_tree();

  model::PointCloud syn_cloud = pertubPointCloud(
      *cloud_prev, mock_trans_x_, mock_trans_y_, mock_trans_z_);

  syn_cloud.initialize_kd_tree();
  model::RegistrationResult result(std::move(syn_cloud));
  estimateTranslation(cloud_prev, &result);
  return result;
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

}  // namespace phaser_core
