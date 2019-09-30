#pragma once

#include "packlo/backend/registration/sph-registration.h"

namespace registration {

class SphRegistrationMockTransformed : public SphRegistration {
	public:
		virtual ~SphRegistrationMockTransformed() = default;
		virtual void registerPointCloud(model::PointCloudPtr cloud_prev, 
				model::PointCloudPtr cloud_cur) override;

	private:
};

} // namespace registration
