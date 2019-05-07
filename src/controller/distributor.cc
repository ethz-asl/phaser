#include <packlo/controller/distributor.h>

#include <glog/logging.h>

namespace controller {

Distributor::Distributor(common::Datasource& ds)
  : ds_(ds) {
  subscribeToTopics();
}

void Distributor::subscribeToTopics() {
  ds_.subscribeToLidarImages([&] (const sensor_msgs::ImageConstPtr& img) {
  VLOG(1) << "RECV IMG2";
    lidarImageCallback(img);
  }); 
  VLOG(1) << "subscribed";
}

void Distributor::lidarImageCallback(const sensor_msgs::ImageConstPtr& img) {
  VLOG(1) << "img finally received";
}

}
