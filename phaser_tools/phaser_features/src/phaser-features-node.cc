#include "phaser/features/phaser-features-node.h"

#include "phaser/backend/registration/sph-opt-registration.h"
#include "phaser/backend/registration/sph-registration.h"

#include <glog/logging.h>
#include <ros/ros.h>

DEFINE_string(
    phaser_features_input_dataset, "",
    "Defines the directory to read the features from.");
DEFINE_string(
    phaser_features_output_dataset, "",
    "Defines the path to the output dataset.");

namespace phaser_features {

PhaserFeaturesNode::PhaserFeaturesNode(
    ros::NodeHandle* nh, ros::NodeHandle* nh_private)
    : spinner_(1), node_handle_(nh), node_handle_private_(nh_private) {
  should_exit_.store(false);

  ds_ = std::make_unique<data::DatasourceFeatures>(
      FLAGS_phaser_features_input_dataset);
  CHECK_NOTNULL(ds_);
  ctrl_ = std::make_unique<PhaserFeatureController>(
      ds_, FLAGS_phaser_features_output_dataset);
  CHECK_NOTNULL(ctrl_);
}

bool PhaserFeaturesNode::run() {
  LOG(INFO) << "--- Running Phaser Feature Extractor "
               "--------------------------------";
  spinner_.start();
  if (ds_ == nullptr) {
    return false;
  }
  ds_->startStreaming();
  return true;
}

const std::atomic<bool>& PhaserFeaturesNode::shouldExit() const noexcept {
  return should_exit_;
}

std::string PhaserFeaturesNode::updateAndPrintStatistics() {
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

void PhaserFeaturesNode::shutdown() {}

}  // namespace phaser_features
