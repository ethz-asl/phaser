#pragma once

#include <ros/ros.h>

namespace packlo {

class PackloNode {
  public:
    explicit PackloNode(const ros::NodeHandle& nh);

  private:
    const ros::NodeHandle& node_handle_;
  
}; // class PackloNode

} // namespace packlo
