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

DEFINE_string(app_mode, "registration", 
    "Defines the operating mode.");

namespace controller {

Distributor::Distributor(data::DatasourcePtr& ds)
    : ds_(ds), statistics_manager_(kManagerReferenceName), 
    registration_algorithm_(FLAGS_registration_algorithm) {
  subscribeToTopics();
  initializeRegistrationAlgorithm();
  ds_->startStreaming();
}

void Distributor::subscribeToTopics() {
  ds_->subscribeToPointClouds(
    [&] (const model::PointCloudPtr& cloud) {
      CHECK(cloud);
      pointCloudCallback(cloud);
  }); 
}

void Distributor::initializeRegistrationAlgorithm() {
  if (registration_algorithm_ == "sph")
    registrator_ = std::make_unique<registration::SphRegistration>();
  else if (registration_algorithm_ == "sph-mock-rotated")
    registrator_ 
      = std::make_unique<registration::SphRegistrationMockRotated>();
  else if (registration_algorithm_ == "sph-mock-cutted")
    registrator_ = std::make_unique<registration::SphRegistrationMockCutted>();
  else if (registration_algorithm_ == "sph-mock-translated")
    registrator_ = std::make_unique<
      registration::SphRegistrationMockTranslated>();
  else if (registration_algorithm_ == "sph-mock-transformed")
    registrator_ = std::make_unique<
      registration::SphRegistrationMockTransformed>();
  else 
    LOG(FATAL) << "Unknown registration algorithm specified!";
}

void Distributor::setRegistrationAlgorithm(std::string&& algorithm) {
  registration_algorithm_ = algorithm;
}

void Distributor::setRegistrationAlgorithm(const std::string& algorithm) {
  registration_algorithm_ = algorithm;
}

// TODO(lbern): should i just pass a different callback
void Distributor::pointCloudCallback(
    const model::PointCloudPtr& cloud) {
  VLOG(1) << "received cloud in callback";
  preprocessPointCloud(cloud);
  if (FLAGS_app_mode == "registration") 
    registerPointCloud(cloud);
  else if(FLAGS_app_mode == "store_ply")
    cloud->writeToFile();
  else
    LOG(FATAL) << "Unknown applicaiton mode. Aborting.";
}

void Distributor::registerPointCloud(const model::PointCloudPtr& cloud) {
  if (prev_point_cloud_ == nullptr) {
    prev_point_cloud_ = cloud;
    return;
  }
  
  CHECK_NOTNULL(registrator_);
  registrator_->registerPointCloud(prev_point_cloud_, cloud);
  prev_point_cloud_ = cloud;
}

void Distributor::preprocessPointCloud(
    const model::PointCloudPtr& cloud) {
  common::PointCloud_tPtr input_cloud = cloud->getRawCloud();

  // Why is this needed?
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
}

void Distributor::updateStatistics() {
  VLOG(1) << "updating registrator";
}

void Distributor::getStatistics(
    common::StatisticsManager* manager) const noexcept{
  registrator_->getStatistics(manager);
  manager->mergeManager(statistics_manager_);
}

}
