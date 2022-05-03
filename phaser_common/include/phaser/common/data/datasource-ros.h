#pragma once

#include <array>
#include <functional>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>

#include "phaser/common/data/base-datasource.h"

namespace data {

class DatasourceRos : public BaseDatasource {
 public:
  explicit DatasourceRos(ros::NodeHandle& nh);

  virtual void subscribeToPointClouds(
      boost::function<void(const model::PointCloudPtr&)> func) override;
  virtual void startStreaming(const uint32_t number_of_clouds = 0) override;

 private:
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  ros::NodeHandle& nh_;
  std::vector<ros::Subscriber> subscribers_;
  bool started_;
  uint32_t number_of_clouds_;
  uint32_t processed_clouds_;
};

}  // namespace data
