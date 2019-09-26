#include "packlo/backend/registration/sph-registration.h"
#include "packlo/common/statistic-utils.h"
#include "packlo/common/rotation-utils.h"

#include <glog/logging.h>

DEFINE_int32(spherical_bandwith, 64, 
		"Defines the bandwith used for the spherical registration.");

namespace registration {

SphRegistration::SphRegistration() 
		: //BaseRegistration(kManagerReferenceName),
		sampler_(FLAGS_spherical_bandwith) {
}

void SphRegistration::registerPointCloud(model::PointCloudPtr cloud_prev, 
		model::PointCloudPtr cloud_cur) {
  cloud_prev->initialize_kd_tree();
  cloud_cur->initialize_kd_tree();

	std::array<double, 3> zyz;
	correlatePointcloud(*cloud_prev, *cloud_cur, &zyz);
	model::PointCloud reg_cloud = common::RotationUtils::RotateAroundZYZCopy(
      *cloud_cur, zyz[2], zyz[1], zyz[0]);
}

void SphRegistration::updateStatistics() {
	statistics_manager_.mergeManager(sph_corr_.getStatistics());
	int test = statistics_manager_.count("signal_values");
}

const common::StatisticsManager& SphRegistration::getStatistics() 
		const noexcept {
	return statistics_manager_;
}

void SphRegistration::correlatePointcloud(
		const model::PointCloud& source, 
		const model::PointCloud& target, 
		std::array<double, 3>* const zyz) {
	CHECK(zyz);
  std::vector<float> f_values, h_values; 

	const double duration_sample_f_ms = common::executeTimedFunction(
			&common::SphericalSampler::sampleUniformly, 
			sampler_, source, &f_values);
	const double duration_sample_h_ms = common::executeTimedFunction(
			&common::SphericalSampler::sampleUniformly, 
			sampler_, target, &h_values);

	const double duration_correlation_ms = common::executeTimedFunction(
			&backend::SphericalCorrelation::correlateSignals, 
			sph_corr_, f_values, h_values, 
			sampler_.getInitializedBandwith(), zyz);

	VLOG(1) << "Registered point cloud.\n"
		<< "Sampling took for f and h: [" << duration_sample_f_ms << "ms," 
		<< duration_sample_h_ms << "ms]. \n"
		<< "Correlation took: " << duration_correlation_ms << "ms.";

	statistics_manager_.emplaceValue(kSampleDurationKey, duration_sample_f_ms);
	statistics_manager_.emplaceValue(kSampleDurationKey, duration_sample_h_ms);
	statistics_manager_.emplaceValue(kCorrelationDurationKey, 
			duration_correlation_ms);
}

} // namespace registration
