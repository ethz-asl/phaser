#include <packlo/common/datasource.h>
#include <glog/logging.h>

namespace common {

Datasource::Datasource() 
    : sync_intensity_images_sub_(nh_, "/img_node/intensity_image", 1),
      sync_range_images_sub_(nh_, "/img_node/range_image", 1),
      sync_noise_images_sub_(nh_, "/img_node/noise_image", 1), 
      image_synchronizer_(1, 
          sync_intensity_images_sub_, 
          sync_range_images_sub_, 
          sync_noise_images_sub_) {
}

void Datasource::subscribeToLidarIntensityImages(
    std::function<void(const sensor_msgs::ImageConstPtr&)> func) {
  boost::function<void(const sensor_msgs::ImageConstPtr&)> callback = 
      [&] (const sensor_msgs::ImageConstPtr &img) {
        func(img);
      };
  ros::Subscriber sub = nh_.subscribe("/img_node/intensity_image", 1, callback); 
  subscribers_.emplace_back(std::move(sub));
}

void Datasource::subscribeToImage(
    std::function<void(const sensor_msgs::ImageConstPtr&)> func) {
  boost::function<void(const sensor_msgs::ImageConstPtr&)> callback = 
      [&] (const sensor_msgs::ImageConstPtr &img) {
        func(img);
      };
  ros::Subscriber sub = nh_.subscribe("/VersaVIS/cam0/image_raw", 1, callback); 
  subscribers_.emplace_back(std::move(sub));
}

void Datasource::subscribeToLidarImages(
    std::function<void(const sensor_msgs::ImageConstPtr&, 
      const sensor_msgs::ImageConstPtr&, 
      const sensor_msgs::ImageConstPtr&)> func) {

  boost::function<void(const sensor_msgs::ImageConstPtr&, 
    const sensor_msgs::ImageConstPtr&, 
    const sensor_msgs::ImageConstPtr&)> callback = 
    [&] (const sensor_msgs::ImageConstPtr& intensity_image, 
        const sensor_msgs::ImageConstPtr& range_image, 
        const sensor_msgs::ImageConstPtr& noise_image) {
      func(intensity_image, range_image, noise_image); 
    }; 
  image_synchronizer_.registerCallback(boost::bind(callback, _1, _2, _3));
}
}
