#ifndef PACKLO_BACKEND_REGISTRATION_G_ICP_REGISTRATION_H_
#define PACKLO_BACKEND_REGISTRATION_G_ICP_REGISTRATION_H_

#include "packlo/backend/registration/base-registration.h"
#include <pcl/registration/gicp.h>

namespace registration {

class GIcpRegistration : public BaseRegistration {
 public:
  GIcpRegistration();
  virtual ~GIcpRegistration() = default;
  model::RegistrationResult registerPointCloud(
      model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur) override;
 private:
   pcl::GeneralizedIterativeClosestPoint<
      common::Point_t, common::Point_t> gicp_;

};

}  // namespace registration

#endif  // PACKLO_BACKEND_REGISTRATION_G_ICP_REGISTRATION_H_
