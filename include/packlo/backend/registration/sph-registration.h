#pragma once

#include "packlo/common/spherical-sampler.h"
#include "packlo/backend/registration/base-registration.h"
#include "packlo/backend/correlation/spherical-correlation.h"
#include "packlo/backend/alignment/base-aligner.h"

#include <array>

namespace registration {

class SphRegistration : public BaseRegistration {
	public:
		SphRegistration();

		virtual ~SphRegistration() = default;
		virtual void registerPointCloud(model::PointCloudPtr cloud_prev, 
				model::PointCloudPtr cloud_cur) override;

		virtual void getStatistics(common::StatisticsManager* manager)
			const noexcept override;

	protected:
		void correlatePointcloud(                                          
       const model::PointCloud& source,                                            
       const model::PointCloud& target,                                            
       std::array<double, 3>* const zyz);	

		backend::SphericalCorrelation sph_corr_; 
		common::SphericalSampler sampler_;
		alignment::BaseAlignerPtr aligner_; 

		// Statistics
		const std::string kSampleDurationKey = "Sampling";
		const std::string kCorrelationDurationKey = "Correlation";
};

} // namespace registration
