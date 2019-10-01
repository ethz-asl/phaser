#include "packlo/controller/distributor.h"
#include "packlo/common/spherical-projection.h"
#include "packlo/common/rotation-utils.h"
#include "packlo/visualization/debug-visualizer.h"
#include "packlo/backend/registration/sph-registration.h"
#include "packlo/backend/registration/mock/sph-registration-mock-rotated.h"
#include "packlo/backend/registration/mock/sph-registration-mock-cutted.h"
#include "packlo/backend/registration/mock/sph-registration-mock-translated.h"
#include "packlo/backend/registration/mock/sph-registration-mock-transformed.h"

#include <glog/logging.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

DEFINE_bool(enable_debug, false, 
		"Enables the debug mode for the registration.");

DEFINE_string(registration_algorithm, "sph", 
		"Defines the used algorithm for the point cloud registration.");

namespace controller {

Distributor::Distributor(common::Datasource& ds)
		: ds_(ds), statistics_manager_(kManagerReferenceName) {
  subscribeToTopics();
	initializeRegistrationAlgorithm(FLAGS_registration_algorithm);
}

void Distributor::subscribeToTopics() {
	ds_.subscribeToPointClouds(
		[&] (const sensor_msgs::PointCloud2ConstPtr& cloud) {
			CHECK(cloud);
			pointCloudCallback(cloud);
	}); 
}

void Distributor::initializeRegistrationAlgorithm(const std::string& type) {
	if (type == "sph")
		registrator_ = std::make_unique<registration::SphRegistration>();
	else if (type == "sph-mock-rotated")
		registrator_ 
			= std::make_unique<registration::SphRegistrationMockRotated>();
	else if (type == "sph-mock-cutted")
		registrator_ = std::make_unique<registration::SphRegistrationMockCutted>();
	else if (type == "sph-mock-translated")
		registrator_ = std::make_unique<
			registration::SphRegistrationMockTranslated>();
	else if (type == "sph-mock-transformed")
		registrator_ = std::make_unique<
			registration::SphRegistrationMockTransformed>();
	else 
		LOG(FATAL) << "Unknown registration algorithm specified!";
}

void Distributor::pointCloudCallback(
		const sensor_msgs::PointCloud2ConstPtr& cloud) {
	common::PointCloud_tPtr input_cloud = preprocessPointCloud(cloud);
	if (prev_point_cloud_ == nullptr) {
		prev_point_cloud_ = std::make_shared<model::PointCloud>(input_cloud);
		return;
	}
	model::PointCloudPtr cur_point_cloud_ 
		= std::make_shared<model::PointCloud>(input_cloud);
	registrator_->registerPointCloud(prev_point_cloud_, cur_point_cloud_);
	prev_point_cloud_ = cur_point_cloud_;
}

common::PointCloud_tPtr Distributor::preprocessPointCloud(
		const sensor_msgs::PointCloud2ConstPtr& cloud) {
  common::PointCloud_tPtr input_cloud (new common::PointCloud_t);

	// Why is this needed?
  pcl::fromROSMsg(*cloud, *input_cloud);
  pcl::PassThrough<common::Point_t> pass;
  pass.setInputCloud (input_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0,0.0);
  pass.setFilterLimitsNegative (true);
  pass.filter (*input_cloud);

	// Only for speedup
  pcl::VoxelGrid<common::Point_t> avg;
  avg.setInputCloud(input_cloud);
  avg.setLeafSize(0.25f, 0.25f, 0.25f);
  avg.filter(*input_cloud);

	return input_cloud;
}

void Distributor::updateStatistics() {
	VLOG(1) << "updating registrator";
	//registrator_->updateStatistics();
}

void Distributor::getStatistics(
		common::StatisticsManager* manager) const noexcept{
	registrator_->getStatistics(manager);
	manager->mergeManager(statistics_manager_);
}

}
