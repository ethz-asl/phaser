#ifndef PHASER_BACKEND_REGISTRATION_MOCK_SPH_REGISTRATION_MOCK_ROTATED_H_
#define PHASER_BACKEND_REGISTRATION_MOCK_SPH_REGISTRATION_MOCK_ROTATED_H_

#include "phaser/backend/registration/sph-registration.h"

namespace phaser_core {

class SphRegistrationMockRotated : public SphRegistration {
 public:
  SphRegistrationMockRotated();
  virtual ~SphRegistrationMockRotated() = default;
  model::RegistrationResult registerPointCloud(
      model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur) override;

  void setRandomRotation(
      const double alpha_rad, const double beta_rad, const double gamma_rad);

 private:
  model::PointCloud pertubPointCloud(
      const model::PointCloud& cloud, const float alpha_rad,
      const float beta_rad, const float gamma_rad);

  double mock_alpha_rad_;
  double mock_beta_rad_;
  double mock_gamma_rad_;
};

}  // namespace phaser_core

#endif  // PHASER_BACKEND_REGISTRATION_MOCK_SPH_REGISTRATION_MOCK_ROTATED_H_
