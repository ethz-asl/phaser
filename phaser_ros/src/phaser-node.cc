#include "phaser/phaser-node.h"

#include <glog/logging.h>
#include <ros/ros.h>

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

PhaserNode::PhaserNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : spinner_(1), node_handle_(nh), node_handle_private_(nh_private) {
  should_exit_.store(false);
  initializeDatasource(FLAGS_datasource);
  CHECK_NOTNULL(ds_);

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
  CHECK_NOTNULL(controller_);

  if (prev_point_cloud_ == nullptr) {
    prev_point_cloud_ = cloud;
    return;
  }

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
