#pragma once

#include <packlo/common/datasource.h>
#include <packlo/backend/optical-flow/lk-tracker.h>
#include <packlo/model/point-cloud.h>

namespace controller {

class Distributor {

  public:
    explicit Distributor(common::Datasource& ds);

  private:
    void subscribeToTopics();
    void lidarImageCallback(const sensor_msgs::ImageConstPtr& img);
    void lidarImagesCallback(const sensor_msgs::ImageConstPtr& intensity,
        const sensor_msgs::ImageConstPtr& range,
        const sensor_msgs::ImageConstPtr& noise);
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);

    model::PointCloud pertubPointCloud(model::PointCloud &cloud,
        const float alpha_rad, const float beta_rad, const float gamma_rad);
    
    common::Datasource& ds_;
    optical_flow::LKTracker tracker_;
};

}
