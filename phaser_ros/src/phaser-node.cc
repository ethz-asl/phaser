#include "phaser/phaser-node.h"
#include <sstream>

#include <glog/logging.h>
#include <ros/ros.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>

#include "phaser/backend/registration/mock/sph-registration-mock-cutted.h"
#include "phaser/backend/registration/mock/sph-registration-mock-rotated.h"
#include "phaser/backend/registration/mock/sph-registration-mock-transformed.h"
#include "phaser/backend/registration/mock/sph-registration-mock-translated.h"
#include "phaser/backend/registration/sph-opt-registration.h"
#include "phaser/backend/registration/sph-registration.h"
#include "phaser/common/data/datasource-ply.h"
#include "phaser/common/data/datasource-ros.h"
#include "phaser/visualization/plotty-visualizer.h"

namespace phaser_ros {

DEFINE_string(datasource, "bag", "Defines the datasource to use for packlo.");

DEFINE_int32(n_clouds_to_process, 0, "As the name suggests.");

DEFINE_string(
    registration_algorithm, "sph",
    "Defines the used algorithm for the point cloud registration.");

static void writePointCloud(
		    const std::string& reg_file, model::PointCloudPtr cloud) {
  CHECK(!reg_file.empty());
  CHECK_NOTNULL(cloud);
  auto pcd = cloud->getRawCloud();

  boost::shared_ptr<std::vector<int>> indices(new std::vector<int>);
  pcl::removeNaNFromPointCloud(*pcd, *indices);
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(pcd);
  extract.setIndices(indices);
  extract.filter(*pcd);


  pcl::io::savePLYFileASCII(reg_file, *pcd);
  LOG(INFO) << "Wrote registered cloud to " << reg_file;
}

PhaserNode::PhaserNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : spinner_(1), node_handle_(nh), node_handle_private_(nh_private) {
  should_exit_.store(false);
  initializeDatasource(FLAGS_datasource);
  CHECK_NOTNULL(ds_);

  if (FLAGS_registration_algorithm == "sph")
    controller_ = std::make_unique<phaser_core::CloudController>("sph-opt");
  subscribeToTopics();
}

void PhaserNode::subscribeToTopics() {
  CHECK_NOTNULL(ds_);
  ds_->subscribeToPointClouds([&](const model::PointCloudPtr& cloud) {
    CHECK(cloud);
    pointCloudCallback(cloud);
  });
}

void PhaserNode::pointCloudCallback(const model::PointCloudPtr& cloud) {
  CHECK_NOTNULL(cloud);

  if (FLAGS_registration_algorithm == "export") {
    static std::size_t counter = 0u;
    std::stringstream ss;
    ss << "./reg_" << counter << ".ply";
    writePointCloud(ss.str(), cloud);
    ++counter;
    return;
  }

  if (prev_point_cloud_ == nullptr) {
    prev_point_cloud_ = cloud;
    return;
  }

  CHECK_NOTNULL(controller_);
  controller_->registerPointCloud(prev_point_cloud_, cloud);
  prev_point_cloud_ = cloud;
}

bool PhaserNode::run() {
  LOG(INFO) << "Running PHASER";
  spinner_.start();
  if (ds_ == nullptr || controller_ == nullptr) {
    return false;
  }
  VLOG(1) << "Loading " << FLAGS_n_clouds_to_process << " clouds.";
  ds_->startStreaming(FLAGS_n_clouds_to_process);
  return true;
}

const std::atomic<bool>& PhaserNode::shouldExit() const noexcept {
  return should_exit_;
}

std::string PhaserNode::updateAndPrintStatistics() {
  return "";
}

void PhaserNode::shutdown() {
  controller_->shutdown();
}

void PhaserNode::initializeDatasource(const std::string& type) {
  if (type == "bag")
    ds_ = std::make_shared<data::DatasourceRos>(node_handle_);
  else if (type == "ply")
    ds_ = std::make_shared<data::DatasourcePly>();
  else
    LOG(FATAL) << "Unknown datasource type specified.";
}

std::vector<common::StatisticsManager> PhaserNode::retrieveStatistics()
    const noexcept {
  std::vector<common::StatisticsManager> managers;

  return managers;
}

}  // namespace phaser_ros
