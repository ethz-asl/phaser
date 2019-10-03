#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <vector>
#include <array>
#include <functional>

namespace common {

class Datasource {
  public:
    explicit Datasource(ros::NodeHandle& nh); 

    void subscribeToPointClouds(
        boost::function<void(const sensor_msgs::PointCloud2ConstPtr&)> func);

  private:
    ros::NodeHandle& nh_;
    std::vector<ros::Subscriber> subscribers_;
};

}
