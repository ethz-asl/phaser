#include <packlo/common/datasource.h>
#include <glog/logging.h>

DEFINE_string(point_cloud_topic, "/os1_cloud_node/points", 
		"Defines the topic name for the point clouds.");

namespace common {

Datasource::Datasource(ros::NodeHandle& nh) 
    : nh_(nh)	{}

void Datasource::subscribeToPointClouds(
    boost::function<void(const sensor_msgs::PointCloud2ConstPtr&)> func) {
  ros::Subscriber sub = nh_.subscribe(FLAGS_point_cloud_topic, 1, func); 
  subscribers_.emplace_back(std::move(sub));
	VLOG(1) << "LiDAR point clouds are subscribed to: " 
		<< FLAGS_point_cloud_topic;
}

}
