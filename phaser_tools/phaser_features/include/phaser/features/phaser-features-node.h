#ifndef PHASER_FEATURES_PHASER_FEATURES_NODE_H_
#define PHASER_FEATURES_PHASER_FEATURES_NODE_H_

#include "phaser/common/data/datasource-features.h"
#include "phaser/features/phaser-features-controller.h"

#include <atomic>
#include <memory>
#include <ros/ros.h>
#include <string>
#include <vector>

namespace phaser_features {

class PhaserFeaturesNode {
 public:
  explicit PhaserFeaturesNode(
      ros::NodeHandle* nh, ros::NodeHandle* nh_private_);
  bool run();
  const std::atomic<bool>& shouldExit() const noexcept;
  std::string updateAndPrintStatistics();
  void shutdown();

 private:
  ros::AsyncSpinner spinner_;
  ros::NodeHandle* node_handle_;
  ros::NodeHandle* node_handle_private_;
  data::DatasourceFeaturesPtr ds_;
  std::atomic<bool> should_exit_;
  std::unique_ptr<PhaserFeatureController> ctrl_;
};

}  // namespace phaser_features

#endif  // PHASER_FEATURES_PHASER_FEATURES_NODE_H_
