#pragma once

#include "packlo/common/data/base-datasource.h"
#include "packlo/common/statistics-manager.h"
#include "packlo/controller/distributor.h"

#include <ros/ros.h>
#include <memory>
#include <vector>
#include <atomic>

namespace packlo {

class PackloNode {
  public:
    explicit PackloNode(ros::NodeHandle& nh, 
        ros::NodeHandle& nh_private_);
    bool run();
    const std::atomic<bool>& shouldExit() const noexcept; 
    std::string updateAndPrintStatistics();
    void shutdown();

  private:
    void initializeDatasource(const std::string& type);
    std::vector<common::StatisticsManager> retrieveStatistics() const noexcept;
    
    ros::AsyncSpinner spinner_;
    ros::NodeHandle& node_handle_;
    ros::NodeHandle& node_handle_private_;
    data::DatasourcePtr ds_;
    std::unique_ptr<controller::Distributor> dist_;
    std::atomic<bool> should_exit_; 
  
}; // class PackloNode

} // namespace packlo
