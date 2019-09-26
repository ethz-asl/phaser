#pragma once

#include "packlo/model/point-cloud.h"
#include "packlo/common/statistics-manager.h"

namespace registration {

class BaseRegistration {
	public:

		virtual ~BaseRegistration() = default;
		virtual void registerPointCloud(model::PointCloudPtr cloud_prev, 
				model::PointCloudPtr cloud_cur) = 0;

		virtual void getStatistics(common::StatisticsManager* manager)
			const noexcept;

	protected:
		BaseRegistration() 
			: statistics_manager_("") {  }
		BaseRegistration(const std::string& reference_name) 
			: statistics_manager_(reference_name) {  }

		common::StatisticsManager statistics_manager_;
};

} // namespace registration
