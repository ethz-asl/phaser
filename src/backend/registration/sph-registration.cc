#include "packlo/backend/registration/sph-registration.h"
#include "packlo/backend/alignment/range-based-aligner.h"
#include "packlo/common/statistic-utils.h"
#include "packlo/common/rotation-utils.h"
#include "packlo/visualization/debug-visualizer.h"

#include <glog/logging.h>

DEFINE_int32(spherical_bandwith, 128, 
		"Defines the bandwith used for the spherical registration.");

namespace registration {

SphRegistration::SphRegistration() 
		: BaseRegistration("SphRegistration"),
		sampler_(FLAGS_spherical_bandwith) {
	aligner_ = std::make_unique<alignment::RangeBasedAligner>();
}

void SphRegistration::registerPointCloud(model::PointCloudPtr cloud_prev, 
		model::PointCloudPtr cloud_cur) {
	CHECK(cloud_prev);
	CHECK(cloud_cur);
  cloud_prev->initialize_kd_tree();
  cloud_cur->initialize_kd_tree();

	std::array<double, 3> zyz;
	correlatePointcloud(*cloud_prev, *cloud_cur, &zyz);
	model::PointCloud reg_cloud = common::RotationUtils::RotateAroundZYZCopy(
      *cloud_cur, zyz[2], zyz[1], zyz[0]);
  visualization::DebugVisualizer::getInstance()
		.visualizePointCloudDiff(*cloud_prev, reg_cloud);

	CHECK(!f_values_.empty());
	CHECK(!h_values_.empty());
	aligner_->alignRegistered(*cloud_prev, f_values_, 
			reg_cloud, h_values_);
}

void SphRegistration::getStatistics(
		common::StatisticsManager* manager) const noexcept {
	BaseRegistration::getStatistics(manager);
	sph_corr_.getStatistics(manager);
}

void SphRegistration::correlatePointcloud(
		const model::PointCloud& source, 
		const model::PointCloud& target, 
		std::array<double, 3>* const zyz) {
	CHECK(zyz);

	const double duration_sample_f_ms = common::executeTimedFunction(
			&common::SphericalSampler::sampleUniformly, 
			&sampler_, source, &f_values_);
	const double duration_sample_h_ms = common::executeTimedFunction(
			&common::SphericalSampler::sampleUniformly, 
			&sampler_, target, &h_values_);
	CHECK(f_values_.size() == h_values_.size());

	const double duration_correlation_ms = common::executeTimedFunction(
			&backend::SphericalCorrelation::correlateSignals, 
			&sph_corr_, f_values_, h_values_, 
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
