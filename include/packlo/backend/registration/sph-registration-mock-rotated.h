#pragma once

#include "packlo/backend/registration/sph-registration.h"

namespace registration {

class SphRegistrationMockRotated : public SphRegistration {
	public:
		virtual ~SphRegistrationMockRotated() = default;
		virtual void registerPointCloud(model::PointCloudPtr cloud_prev, 
				model::PointCloudPtr cloud_cur) override;
	private:
		model::PointCloud pertubPointCloud(model::PointCloud &cloud,
				const float alpha_rad, const float beta_rad, const float gamma_rad);
};

} // namespace registration
