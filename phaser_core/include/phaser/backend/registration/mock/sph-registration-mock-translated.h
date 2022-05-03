#pragma once

#include "phaser/backend/registration/sph-registration.h"

namespace phaser_core {

class SphRegistrationMockTranslated : public SphRegistration {
 public:
  SphRegistrationMockTranslated();
  virtual ~SphRegistrationMockTranslated() = default;
  virtual model::RegistrationResult registerPointCloud(
      model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur) override;

  void setRandomTranslation(
      const double mock_trans_x, const double mock_trans_y,
      const double mock_trans_z);

 private:
  model::PointCloud pertubPointCloud(
      model::PointCloud& cloud, const float x, const float y, const float z);

  std::vector<model::FunctionValue> pertubFunctionValues(
      std::vector<model::FunctionValue>& values, const float x, const float y,
      const float z);

  double mock_trans_x_;
  double mock_trans_y_;
  double mock_trans_z_;
};

}  // namespace phaser_core
