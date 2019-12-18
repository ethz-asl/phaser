#ifndef PACKLO_ROS_COMMON_GFLAGS_INTERFACE_H_
#define PACKLO_ROS_COMMON_GFLAGS_INTERFACE_H_

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

namespace common {

void parseGflagsFromRosParams(
    const char* program_name, const ros::NodeHandle& nh_private);

}  // namespace common

#endif  // PACKLO_ROS_COMMON_GFLAGS_INTERFACE_H_
