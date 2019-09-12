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

    void subscribeToLidarIntensityImages(
        std::function<void(const sensor_msgs::ImageConstPtr&)> func);
    void subscribeToImage(
        std::function<void(const sensor_msgs::ImageConstPtr&)> func);
    void subscribeToLidarImages(
        std::function<void(const sensor_msgs::ImageConstPtr&, 
          const sensor_msgs::ImageConstPtr&, 
          const sensor_msgs::ImageConstPtr&)> func);

    void subscribeToPointClouds(
        boost::function<void(const sensor_msgs::PointCloud2ConstPtr&)> func);

  private:
    ros::NodeHandle& nh_;
    std::vector<ros::Subscriber> subscribers_;

    using SyncedImageSubscriber 
      = message_filters::Subscriber<sensor_msgs::Image>;
    using ImagePolicy 
      = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image>;
    using ImageSynchronizer = message_filters::Synchronizer<ImagePolicy>;
    

    SyncedImageSubscriber sync_intensity_images_sub_;
    SyncedImageSubscriber sync_range_images_sub_;
    SyncedImageSubscriber sync_noise_images_sub_;
    ImageSynchronizer image_synchronizer_;

};

}
