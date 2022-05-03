#include <chrono>
#include <glog/logging.h>
#include <ros/ros.h>
#include <thread>

#include "phaser/common/gflags-interface.h"
#include "phaser/phaser-node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "phaser");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh, nh_private("~");
  common::parseGflagsFromRosParams(argv[0], nh_private);

  phaser_ros::PhaserNode phaser_node(nh, nh_private);
  if (!phaser_node.run()) {
    ROS_FATAL("Failed to start running the phaser node!");
    ros::shutdown();
    return 1;
  }

  const std::atomic<bool>& end_of_days_signal_received =
      phaser_node.shouldExit();
  while (ros::ok() && !end_of_days_signal_received.load()) {
    VLOG_EVERY_N(1, 10) << "\n" << phaser_node.updateAndPrintStatistics();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  phaser_node.shutdown();
  return 0;
}
