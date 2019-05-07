#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include <functional>

namespace common {

class Datasource {
  public:
    
    void subscribeToLidarImages(
        std::function<void(const sensor_msgs::ImageConstPtr&)> func);

  private:
    ros::NodeHandle nh_;
    std::vector<ros::Subscriber> subscribers_;
};

}
