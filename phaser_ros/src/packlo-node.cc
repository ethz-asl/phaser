#include "phaser/packlo-node.h"
#include "phaser/common/data/datasource-ros.h"
#include "phaser/common/data/datasource-ply.h"
#include "phaser/controller/distributor.h"
#include "phaser/visualization/plotty-visualizer.h"

#include <glog/logging.h>
#include <ros/ros.h>

namespace packlo {

DEFINE_string(datasource, "bag",
  "Defines the datasource to use for packlo.");

PackloNode::PackloNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : spinner_(1), node_handle_(nh), node_handle_private_(nh_private) {
  should_exit_.store(false);
  initializeDatasource(FLAGS_datasource);

  CHECK_NOTNULL(ds_);
  dist_ = std::make_unique<controller::Distributor>(ds_);
}

bool PackloNode::run() {
  LOG(INFO) << "Running PackLO";
  spinner_.start();
  return dist_.get() != nullptr;
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
    LOG(FATAL) << "";
}

std::vector<common::StatisticsManager> PackloNode::retrieveStatistics()
    const noexcept {
  std::vector<common::StatisticsManager> managers;
  //  managers.emplace_back(dist_->getStatistics());

  return managers;
}

}  // namespace packlo
