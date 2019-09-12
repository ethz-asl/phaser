#pragma once

#include "packlo/common/spherical-sampler.h"
#include "packlo/common/statistics-manager.h"
#include "packlo/backend/registration/base-registration.h"
#include "packlo/backend/correlation/spherical-correlation.h"

#include <array>

namespace registration {

class SphRegistration : public BaseRegistration {
	public:
		SphRegistration();

		virtual ~SphRegistration() = default;
		virtual void registerPointCloud(model::PointCloudPtr cloud_prev, 
				model::PointCloudPtr cloud_cur) override;

		const common::StatisticsManager& getStatistics() const noexcept;
	
	protected:
		void correlatePointcloud(                                          
       const model::PointCloud& source,                                            
       const model::PointCloud& target,                                            
       std::array<double, 3>* const zyz);	

		backend::SphericalCorrelation sph_corr_; 
		common::SphericalSampler sampler_;

		// Statistics
		const std::string kManagerReferenceName = "SphRegistration";
		const std::string kSampleDurationKey = "Sampling";
		const std::string kCorrelationDurationKey = "Correlation";
		common::StatisticsManager statistics_manager_;
};

} // namespace registration