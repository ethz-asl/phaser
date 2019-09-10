#pragma once

#include <packlo/common/datasource.h>
#include <packlo/model/point-cloud.h>
#include <packlo/backend/correlation/spherical-correlation.h>

#include <array>

namespace controller {

class Distributor {

  public:
    explicit Distributor(common::Datasource& ds);

  private:
    void subscribeToTopics();
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);

    model::PointCloud pertubPointCloud(model::PointCloud &cloud,
        const float alpha_rad, const float beta_rad, const float gamma_rad);
		std::array<double, 3> correlatePointcloud(
				const model::PointCloud& source, 
				const model::PointCloud& target, 
				const int bandwith); 

    
    common::Datasource& ds_;
  backend::SphericalCorrelation sph_corr_; 
};

}
