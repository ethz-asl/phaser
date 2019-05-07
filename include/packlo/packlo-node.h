#pragma once

#include <packlo/common/datasource.h>
#include <packlo/controller/distributor.h>

#include <ros/ros.h>
#include <memory>

namespace packlo {

class PackloNode {
  public:
    explicit PackloNode(const ros::NodeHandle& nh);

  private:
    const ros::NodeHandle& node_handle_;
    common::Datasource ds_;
    std::unique_ptr<controller::Distributor> dist_;
  
}; // class PackloNode

} // namespace packlo
