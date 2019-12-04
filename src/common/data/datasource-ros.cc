#include "packlo/common/data/datasource-ros.h"
#include <pcl_conversions/pcl_conversions.h>
#include <glog/logging.h>

DEFINE_string(
    point_cloud_topic, "/os1_cloud_node/points",
    "Defines the topic name for the point clouds.");

namespace data {

DatasourceRos::DatasourceRos(ros::NodeHandle& nh) : nh_(nh), started_(false) {}

void DatasourceRos::subscribeToPointClouds(
    boost::function<void(const model::PointCloudPtr& cloud)> func) {
  // Create local callback.
  boost::function<void(const sensor_msgs::PointCloud2ConstPtr&)> callback =
      boost::bind(&DatasourceRos::pointCloudCallback, this, _1);
  ros::Subscriber sub = nh_.subscribe(FLAGS_point_cloud_topic, 1, callback);

  subscribers_.emplace_back(std::move(sub));
  callbacks_.emplace_back(func);
  VLOG(1) << "LiDAR point clouds are subscribed to: "
          << FLAGS_point_cloud_topic;
}

void DatasourceRos::startStreaming(const uint32_t number_of_clouds) {
  number_of_clouds_ = number_of_clouds;
  started_ = true;
}

void DatasourceRos::pointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& msg) {
  if (!started_) return;
  if (number_of_clouds_ != 0 && ++processed_clouds_ > number_of_clouds_)
    return;

  common::ExtractedPointCloud_tPtr input_cloud(
      new common::ExtractedPointCloud_t);

  pcl::fromROSMsg(*msg, *input_cloud);
  model::PointCloudPtr cur_cloud =
      std::make_shared<model::PointCloud>(input_cloud);

  for (auto& func : callbacks_) {
    func(cur_cloud);
  }
}

}  // namespace data
