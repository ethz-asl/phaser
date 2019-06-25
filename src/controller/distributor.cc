#include <packlo/controller/distributor.h>
#include <packlo/common/spherical-projection.h>
#include <packlo/common/spherical-sampler.h>
#include <packlo/common/rotation-utils.h>
#include <packlo/visualization/debug-visualizer.h>
#include <packlo/backend/correlation/spherical-correlation.h>

#include <glog/logging.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

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
  VLOG(1) << "Received point cloud";
  
  model::PointCloud_tPtr input_cloud (new model::PointCloud_t);
  pcl::fromROSMsg(*cloud, *input_cloud);
  model::PointCloud point_cloud (input_cloud);
  model::PointCloud syn_cloud = pertubPointCloud(point_cloud);

  common::SphericalProjection::convertPointCloud(point_cloud);
  common::SphericalProjection::convertPointCloud(syn_cloud);


  const int bw = 32;

  std::vector<float> f_values = common::SphericalSampler::sampleUniformly(point_cloud, bw);
  //visualization::DebugVisualizer::getInstance().writeFunctionValuesToFile("F1.txt", f_values);  

  std::vector<float> h_values = common::SphericalSampler::sampleUniformly(syn_cloud, bw);
  //visualization::DebugVisualizer::getInstance().writeFunctionValuesToFile("F2.txt", f_values);  

  backend::SphericalCorrelation sph_corr; 
  std::array<double, 3> zyz = sph_corr.correlateSignals(f_values, h_values, bw);

  VLOG(1) << "Done processing point cloud";
  
}

model::PointCloud Distributor::pertubPointCloud(model::PointCloud &cloud) {
  const float alpha_rad = 0.0f;
  const float beta_rad = 0.0f;
  const float gamma_rad = M_PI/4;

  return common::RotationUtils::RotateAroundXYZCopy(
      cloud, alpha_rad, beta_rad, gamma_rad);
}

}
