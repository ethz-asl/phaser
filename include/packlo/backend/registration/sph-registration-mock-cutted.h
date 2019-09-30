#pragma once

#include "packlo/backend/registration/sph-registration.h"

namespace registration {

class SphRegistrationMockCutted : public SphRegistration {
	public:
		virtual ~SphRegistrationMockCutted() = default;
		virtual void registerPointCloud(model::PointCloudPtr cloud_prev, 
				model::PointCloudPtr cloud_cur) override;

	private:
		model::PointCloud cutPointCloud(common::PointCloud_tPtr& cloud, 
				double min, double max, std::string&& dim);
};

} // namespace handler
