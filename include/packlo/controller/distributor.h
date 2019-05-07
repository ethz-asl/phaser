#pragma once

#include <packlo/common/datasource.h>

namespace controller {

class Distributor {

  public:
    explicit Distributor(common::Datasource& ds);

  private:
    void subscribeToTopics();
    void lidarImageCallback(const sensor_msgs::ImageConstPtr& img);
    
    common::Datasource& ds_;
};

}
