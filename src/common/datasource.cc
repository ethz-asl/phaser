#include <packlo/common/datasource.h>
#include <glog/logging.h>

namespace common {

void Datasource::subscribeToLidarImages(
    std::function<void(const sensor_msgs::ImageConstPtr&)> func) {
  boost::function<void(const sensor_msgs::ImageConstPtr&)> callback = 
      [&] (const sensor_msgs::ImageConstPtr &img) {
        func(img);
      };
  ros::Subscriber sub = nh_.subscribe("/img_node/intensity_image", 1, callback); 
  subscribers_.emplace_back(std::move(sub));
}


}
