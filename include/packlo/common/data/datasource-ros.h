#pragma once

#include "packlo/common/data/base-datasource.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <vector>
#include <array>
#include <functional>

namespace data {

class DatasourceRos : public BaseDatasource {
  public:
    explicit DatasourceRos(ros::NodeHandle& nh); 

    virtual void subscribeToPointClouds(
        boost::function<void(const model::PointCloudPtr&)> func) override;
		virtual void startStreaming() override;
  private:
		void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

    ros::NodeHandle& nh_;
    std::vector<ros::Subscriber> subscribers_;
		bool started_;
};

}
