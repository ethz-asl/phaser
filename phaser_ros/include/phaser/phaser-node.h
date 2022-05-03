#ifndef PHASER_PHASER_NODE_H_
#define PHASER_PHASER_NODE_H_

#include <atomic>
#include <memory>
#include <ros/ros.h>
#include <string>
#include <vector>

#include "phaser/common/data/base-datasource.h"
#include "phaser/common/statistics-manager.h"
#include "phaser/controller/cloud-controller.h"
#include "phaser/model/point-cloud.h"

namespace phaser_ros {

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
  std::vector<common::StatisticsManager> retrieveStatistics() const noexcept;
  void subscribeToTopics();
  void pointCloudCallback(const model::PointCloudPtr& cloud);

  ros::AsyncSpinner spinner_;
  ros::NodeHandle& node_handle_;
  ros::NodeHandle& node_handle_private_;
  data::DatasourcePtr ds_;
  std::atomic<bool> should_exit_;
  std::unique_ptr<phaser_core::CloudController> controller_;
  model::PointCloudPtr prev_point_cloud_;
};  // class PhaserNode

}  // namespace phaser_ros

#endif  // PHASER_PHASER_NODE_H_
