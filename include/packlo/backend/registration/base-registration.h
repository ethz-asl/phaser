#pragma once

#include "packlo/model/point-cloud.h"

namespace registration {

class BaseRegistration {
	public:
		virtual ~BaseRegistration() = default;
		virtual void registerPointCloud(model::PointCloudPtr cloud_prev, 
				model::PointCloudPtr cloud_cur) = 0;

};

} // namespace registration
