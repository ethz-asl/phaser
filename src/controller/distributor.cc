#include <packlo/controller/distributor.h>

#include <glog/logging.h>

namespace controller {

Distributor::Distributor(common::Datasource& ds)
  : ds_(ds) {
  subscribeToTopics();
}

void Distributor::subscribeToTopics() {
  /*
  const bool image = false;
  if (image) 
    ds_.subscribeToImage(
      [&] (const sensor_msgs::ImageConstPtr& img) {
        lidarImageCallback(img);
      }); 
  else
    ds_.subscribeToLidarImages(
      [&] (const sensor_msgs::ImageConstPtr& intensity_image, 
          const sensor_msgs::ImageConstPtr& range_image, 
          const sensor_msgs::ImageConstPtr& noise_image) {
        lidarImagesCallback(intensity_image, range_image, noise_image);
      });
      */
    ds_.subscribeToPointClouds(
      [&] (const sensor_msgs::PointCloud2ConstPtr& img) {
        pointCloudCallback(img);
      }); 

}

void Distributor::lidarImageCallback(const sensor_msgs::ImageConstPtr& img) {
  tracker_.trackNewImage(img);
}

void Distributor::lidarImagesCallback(
    const sensor_msgs::ImageConstPtr& intensity,
    const sensor_msgs::ImageConstPtr& range,
    const sensor_msgs::ImageConstPtr& noise) {
  tracker_.trackNewImages(intensity, range, noise);
}

void Distributor::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud) {
  VLOG(1) << "Received Point cloud";
}

}
