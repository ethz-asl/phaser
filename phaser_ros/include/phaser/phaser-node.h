#ifndef PHASER_PHASER_NODE_H_
#define PHASER_PHASER_NODE_H_

#include "phaser/common/data/base-datasource.h"
#include "phaser/common/statistics-manager.h"
#include "phaser/controller/distributor.h"

#include <atomic>
#include <memory>
#include <ros/ros.h>
#include <string>
#include <vector>

namespace packlo {

class PhaserNode {
 public:
  explicit PhaserNode(
      ros::NodeHandle& nh, ros::NodeHandle& nh_private_);  // NOLINT
  bool run();
  const std::atomic<bool>& shouldExit() const noexcept;
  std::string updateAndPrintStatistics();
  void shutdown();

 private:
  void initializeDatasource(const std::string& type);
  phaser_core::BaseRegistrationPtr initializeRegistrationAlgorithm(
      const std::string& algo);
  std::vector<common::StatisticsManager> retrieveStatistics() const noexcept;

  ros::AsyncSpinner spinner_;
  ros::NodeHandle& node_handle_;
  ros::NodeHandle& node_handle_private_;
  data::DatasourcePtr ds_;
  std::atomic<bool> should_exit_;
  std::unique_ptr<phaser_core::Distributor> dist_;
};  // class PhaserNode

}  // namespace packlo

#endif  // PHASER_PHASER_NODE_H_
