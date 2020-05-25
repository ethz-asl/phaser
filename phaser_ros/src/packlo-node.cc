#include "phaser/packlo-node.h"
#include "phaser/backend/registration/mock/sph-registration-mock-cutted.h"
#include "phaser/backend/registration/mock/sph-registration-mock-rotated.h"
#include "phaser/backend/registration/mock/sph-registration-mock-transformed.h"
#include "phaser/backend/registration/mock/sph-registration-mock-translated.h"
#include "phaser/backend/registration/sph-opt-registration.h"
#include "phaser/backend/registration/sph-registration.h"
#include "phaser/common/data/datasource-ply.h"
#include "phaser/common/data/datasource-ros.h"
#include "phaser/controller/distributor.h"
#include "phaser/visualization/plotty-visualizer.h"

#include <glog/logging.h>
#include <ros/ros.h>

namespace packlo {

DEFINE_string(datasource, "bag", "Defines the datasource to use for packlo.");

DEFINE_int32(n_clouds_to_process, 0, "As the name suggests.");

DEFINE_string(
    registration_algorithm, "sph",
    "Defines the used algorithm for the point cloud registration.");

PackloNode::PackloNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : spinner_(1), node_handle_(nh), node_handle_private_(nh_private) {
  should_exit_.store(false);
  initializeDatasource(FLAGS_datasource);
  phaser_core::BaseRegistrationPtr reg =
      initializeRegistrationAlgorithm(FLAGS_registration_algorithm);

  CHECK_NOTNULL(ds_);
  CHECK_NOTNULL(reg);
  dist_ = std::make_unique<phaser_core::Distributor>(ds_, std::move(reg));
}

bool PackloNode::run() {
  LOG(INFO) << "Running PackLO";
  spinner_.start();
  if (ds_ == nullptr || dist_ == nullptr) {
    return false;
  }
  VLOG(1) << "Loading " << FLAGS_n_clouds_to_process << " clouds.";
  ds_->startStreaming(FLAGS_n_clouds_to_process);
  return true;
}

const std::atomic<bool>& PackloNode::shouldExit() const noexcept {
  return should_exit_;
}

std::string PackloNode::updateAndPrintStatistics() {
  /*
  std::vector<common::StatisticsManager> managers = retrieveStatistics();
  for (common::StatisticsManager manager : managers) {
  }
  */
  // dist_->updateStatistics();
  /*
  common::StatisticsManager manager("main");
  dist_->getStatistics(&manager);
  visualization::PlottyVisualizer::getInstance()
    .createPlotFor(manager, "signal_values");
  */

  return "";
}

void PackloNode::shutdown() {
  dist_->shutdown();
}

void PackloNode::initializeDatasource(const std::string& type) {
  if (type == "bag")
    ds_ = std::make_shared<data::DatasourceRos>(node_handle_);
  else if (type == "ply")
    ds_ = std::make_shared<data::DatasourcePly>();
  else
    LOG(FATAL) << "Unknown datasource type specified.";
}

std::vector<common::StatisticsManager> PackloNode::retrieveStatistics() const
    noexcept {
  std::vector<common::StatisticsManager> managers;
  //  managers.emplace_back(dist_->getStatistics());

  return managers;
}

phaser_core::BaseRegistrationPtr PackloNode::initializeRegistrationAlgorithm(
    const std::string& algo) {
  if (algo == "sph")
    return std::make_unique<phaser_core::SphRegistration>();
  else if (algo == "sph-opt")
    return std::make_unique<phaser_core::SphOptRegistration>();
  else if (algo == "sph-mock-rotated")
    return std::make_unique<phaser_core::SphRegistrationMockRotated>();
  else if (algo == "sph-mock-cutted")
    return std::make_unique<phaser_core::SphRegistrationMockCutted>();
  else if (algo == "sph-mock-translated")
    return std::make_unique<phaser_core::SphRegistrationMockTranslated>();
  else if (algo == "sph-mock-transformed")
    return std::make_unique<phaser_core::SphRegistrationMockTransformed>();
  else
    LOG(FATAL) << "Unknown registration algorithm specified!";
  /*
  if (FLAGS_app_mode == "experiment1" || FLAGS_app_mode == "experiment2" ||
      FLAGS_app_mode == "experiment3") {
    experiment_handler_ = std::make_unique<ExperimentHandler>();
  }
  */
  return nullptr;
}

}  // namespace packlo
