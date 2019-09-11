#include "packlo/controller/distributor.h"
#include "packlo/common/spherical-projection.h"
#include "packlo/common/rotation-utils.h"
#include "packlo/visualization/debug-visualizer.h"
#include "packlo/common/statistic-utils.h"

#include <glog/logging.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>

DEFINE_bool(enable_debug, false, 
		"Enables the debug mode for the registration.");
DEFINE_int32(spherical_bandwith, 64, 
		"Defines the bandwith used for the spherical registration.");

namespace controller {

Distributor::Distributor(common::Datasource& ds)
		: ds_(ds), sampler_(FLAGS_spherical_bandwith), 
		statistics_manager_(kManagerReferenceName) {
  subscribeToTopics();
}

void Distributor::subscribeToTopics() {
	ds_.subscribeToPointClouds(
		[&] (const sensor_msgs::PointCloud2ConstPtr& cloud) {
			CHECK(cloud);
			pointCloudCallback(cloud);
	}); 
}

void Distributor::pointCloudCallback(
		const sensor_msgs::PointCloud2ConstPtr& cloud) {
  VLOG(1) << "Received point cloud";
  model::PointCloud_tPtr input_cloud (new model::PointCloud_t);
  pcl::fromROSMsg(*cloud, *input_cloud);
  pcl::PassThrough<model::Point_t> pass;
  pass.setInputCloud (input_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0,0.0);
  pass.setFilterLimitsNegative (true);
  pass.filter (*input_cloud);

  pcl::VoxelGrid<model::Point_t> avg;
  avg.setInputCloud(input_cloud);
  avg.setLeafSize(0.25f, 0.25f, 0.25f);
  avg.filter(*input_cloud);

	model::Point_t min_pt, max_pt;
	pcl::getMinMax3D(*input_cloud, min_pt, max_pt);
  model::PointCloud_tPtr input_cloud2 (new model::PointCloud_t);
	pcl::copyPointCloud(*input_cloud, *input_cloud2);

	float dist = std::sqrt((min_pt.y - max_pt.y) * (min_pt.y - max_pt.y));
	VLOG(1) << "min pt dist: " << dist;
  model::PointCloud point_cloud = cutPointCloud(input_cloud,
			min_pt.y + 0.2 * dist, min_pt.y + 0.60 * dist, "y");
  model::PointCloud syn_cloud_init = cutPointCloud(input_cloud2,
			min_pt.y + 0.50 * dist, min_pt.y + dist, "y");

 // model::PointCloud point_cloud (input_cloud);
  point_cloud.initialize_kd_tree();

  const float alpha_rad = M_PI/2.0f;
  const float beta_rad = M_PI/2.2f;
  const float gamma_rad = M_PI/2.5f;

  model::PointCloud syn_cloud = pertubPointCloud(syn_cloud_init, 
      alpha_rad, beta_rad, gamma_rad);
  syn_cloud.initialize_kd_tree();

	if (FLAGS_enable_debug) {
		visualization::DebugVisualizer::getInstance()
			.visualizePointCloudDiff(point_cloud, point_cloud);  
		visualization::DebugVisualizer::getInstance()
			.visualizePointCloudDiff(syn_cloud, syn_cloud);  
		visualization::DebugVisualizer::getInstance()
			.visualizePointCloudDiff(point_cloud, syn_cloud);  
	}
  /*
  model::PointCloud cur_sphere = 
    common::SphericalProjection::convertPointCloudCopy(point_cloud);
  model::PointCloud syn_sphere = 
    common::SphericalProjection::convertPointCloudCopy(syn_cloud);
    */

	std::array<double, 3> zyz;
	const double duration_ms = common::executeTimedFunction(
			&Distributor::correlatePointcloud, 
			this, point_cloud, syn_cloud, &zyz);

  VLOG(1) << "Done processing point cloud. it took " 
    << duration_ms
    << "ms";
  
  model::PointCloud reg_cloud = common::RotationUtils::RotateAroundZYZCopy(
      syn_cloud, zyz[2], zyz[1], zyz[0]);
	if (FLAGS_enable_debug)
		visualization::DebugVisualizer::getInstance()
			.visualizePointCloudDiff(point_cloud, reg_cloud);  
}

model::PointCloud Distributor::cutPointCloud(model::PointCloud_tPtr& cloud, 
		double min, double max, std::string&& dim) {
	VLOG(1) << "pass using " << min << " and " << max;
  model::PointCloud_tPtr mod_cloud (new model::PointCloud_t);
  pcl::PassThrough<model::Point_t> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName (dim);
  pass.setFilterLimits (min,max);
  pass.filter (*mod_cloud);
  return model::PointCloud{mod_cloud};
}

model::PointCloud Distributor::pertubPointCloud(model::PointCloud &cloud, 
    const float alpha_rad, const float beta_rad, const float gamma_rad) {
  return common::RotationUtils::RotateAroundXYZCopy(
      cloud, alpha_rad, beta_rad, gamma_rad);
}

void Distributor::correlatePointcloud(
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

	//sampler_.sampleUniformly(source, &f_values);
	//sampler_.sampleUniformly(target, &h_values);

	const double duration_correlation_ms = common::executeTimedFunction(
			&backend::SphericalCorrelation::correlateSignals, 
			sph_corr_, f_values, h_values, 
			sampler_.getInitializedBandwith(), zyz);

  //sph_corr_.correlateSignals(f_values, h_values, 
			//sampler_.getInitializedBandwith(), zyz);
	VLOG(1) << "Registered point cloud.\n"
		<< "Sampling took for f and h: [" << duration_sample_f_ms << "ms," 
		<< duration_sample_h_ms << "ms]. \n"
		<< "Correlation took: " << duration_correlation_ms << "ms.";

	statistics_manager_.emplaceValue(kSampleDurationKey, duration_sample_f_ms);
	statistics_manager_.emplaceValue(kSampleDurationKey, duration_sample_h_ms);
	statistics_manager_.emplaceValue(kCorrelationDurationKey, 
			duration_correlation_ms);
}

const common::StatisticsManager& Distributor::getStatistics() const noexcept{
	return statistics_manager_;
}

}
