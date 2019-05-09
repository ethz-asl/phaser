#pragma once

#include <packlo/common/datasource.h>
#include <packlo/backend/optical-flow/lk-tracker.h>

namespace controller {

class Distributor {

  public:
    explicit Distributor(common::Datasource& ds);

  private:
    void subscribeToTopics();
    void lidarImageCallback(const sensor_msgs::ImageConstPtr& img);
    
    common::Datasource& ds_;
    optical_flow::LKTracker tracker_;
};

}
