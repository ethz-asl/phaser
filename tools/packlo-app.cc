#include <packlo/packlo-node.h>
#include <packlo/controller/distributor.h>

#include <ros/ros.h>
#include <glog/logging.h>

#include <maplab-ros-common/gflags-interface.h>                                 

#include <thread>
#include <chrono>

int main(int argc, char** argv) {
  ros::init(argc, argv, "packlo");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh, nh_private("~");
  ros_common::parseGflagsFromRosParams(argv[0], nh_private);
  
  packlo::PackloNode packlo_node(nh, nh_private);
  if (!packlo_node.run()) {
    ROS_FATAL("Failed to start running the packlo node!");
    ros::shutdown();
    return 1;
	}

  const std::atomic<bool>& end_of_days_signal_received 
		= packlo_node.shouldExit();
  while (ros::ok() && !end_of_days_signal_received.load()) {
		VLOG_EVERY_N(1, 10) << "\n" << packlo_node.updateAndPrintStatistics();
    std::this_thread::sleep_for(std::chrono::seconds(1));
	}

  packlo_node.shutdown();
  return 0;
}
