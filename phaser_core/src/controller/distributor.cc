#include "phaser/controller/distributor.h"
#include "phaser/common/rotation-utils.h"
#include "phaser/common/spherical-projection.h"
#include "phaser/common/statistic-utils.h"

#include <glog/logging.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl_conversions/pcl_conversions.h>

DEFINE_bool(
    enable_debug, false, "Enables the debug mode for the registration.");

DEFINE_string(app_mode, "registration", "Defines the operating mode.");

DEFINE_int32(take_every_n_cloud, 1, "As the name suggests.");

namespace controller {

Distributor::Distributor(const data::DatasourcePtr& ds,
    registration::BaseRegistrationPtr&& registration)
    : ds_(ds),
      statistics_manager_(kManagerReferenceName),
      registrator_(std::move(registration)) {
  subscribeToTopics();
}

void Distributor::shutdown() {
  //writeResultsToFile();
  if (experiment_handler_ != nullptr)
    experiment_handler_->shutdown();
}

void Distributor::subscribeToTopics() {
  ds_->subscribeToPointClouds([&](const model::PointCloudPtr& cloud) {
    CHECK(cloud);
    pointCloudCallback(cloud);
  });
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
  ++cloud_counter_;
  if (cloud_counter_ % FLAGS_take_every_n_cloud != 0) {
    return;
  }
  //preprocessPointCloud(cloud);
  if (FLAGS_app_mode == "registration") {
    if (prev_point_cloud_ == nullptr) {
      prev_point_cloud_ = cloud;
      return;
    }
    const model::RegistrationResult result = registerPointCloud(cloud);
    // prev_point_cloud_ = result.getRegisteredCloud();
    prev_point_cloud_ = nullptr;
    appendResult(result);
    //writeResultsToFile();
  } else if (FLAGS_app_mode == "store_ply")
    cloud->writeToFile();
  else if (FLAGS_app_mode == "experiment1")
    experiment_handler_->runExperiment1(cloud);
  else if (FLAGS_app_mode == "experiment3")
    experiment_handler_->runExperiment3(cloud);
  else
    LOG(FATAL) << "Unknown applicaiton mode. Aborting.";
}

model::RegistrationResult Distributor::registerPointCloud(
    const model::PointCloudPtr& cloud) {
  CHECK_NOTNULL(registrator_);
  CHECK_NOTNULL(prev_point_cloud_);
  /*
  const double duration_sample_f_ms = common::executeTimedFunction(
      &registration::BaseRegistration::registerPointCloud, &(*registrator_),
      prev_point_cloud_, cloud);
  VLOG(1) << "Full registration took: " << duration_sample_f_ms << "ms.";
      */
  model::RegistrationResult result =
      registrator_->registerPointCloud(prev_point_cloud_, cloud);
  // registrator_->registerPointCloud(prev_point_cloud_, cloud);
  return result;
}

void Distributor::preprocessPointCloud(
    const model::PointCloudPtr& cloud) {
  common::PointCloud_tPtr input_cloud = cloud->getRawCloud();
  // Why is this needed?
  pcl::PassThrough<common::Point_t> pass;
  pass.setInputCloud(input_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-2.0, -0.4);
  pass.setFilterLimitsNegative(true);
  pass.filter(*input_cloud);

  // Only for speedup
  pcl::VoxelGrid<common::Point_t> avg;
  avg.setInputCloud(input_cloud);
  avg.setLeafSize(0.25f, 0.25f, 0.25f);
  avg.filter(*input_cloud);
}

void Distributor::updateStatistics() {
  VLOG(1) << "updating registrator";
}

void Distributor::getStatistics(common::StatisticsManager* manager) const
    noexcept {
  registrator_->getStatistics(manager);
  manager->mergeManager(statistics_manager_);
}

void Distributor::appendResult(const model::RegistrationResult& result) {
  const Eigen::VectorXd dq = result.getStateAsVec();
  states_.emplace_back(dq);
}

void Distributor::writeResultsToFile() {
  static std::string filename = "sph-results.txt";
  VLOG(1) << "Writing experiment results to " << filename;
  std::ofstream out_file(filename);
  for (const Eigen::VectorXd& vec : states_) {
    const uint8_t n_vec = 8;
    for (uint8_t i = 0u; i < n_vec; ++i) {
      if (i == n_vec - 1)
        out_file << vec[i] << "\n";
      else
        out_file << vec[i] << ",";
    }
  }
}

}  // namespace controller
