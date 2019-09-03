#include <packlo/controller/distributor.h>
#include <packlo/common/spherical-projection.h>
#include <packlo/common/spherical-sampler.h>
#include <packlo/common/rotation-utils.h>
#include <packlo/visualization/debug-visualizer.h>
#include <packlo/backend/correlation/spherical-correlation.h>

#include <glog/logging.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include <chrono>

namespace controller {

Distributor::Distributor(common::Datasource& ds)
  : ds_(ds) {
  subscribeToTopics();
}

void Distributor::subscribeToTopics() {
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
  /*
    ds_.subscribeToPointClouds(
      [&] (const sensor_msgs::PointCloud2ConstPtr& img) {
        pointCloudCallback(img);
      }); 
      */

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
  pcl::PassThrough<model::Point_t> pass;
  pass.setInputCloud (input_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0,0.0);
  pass.setFilterLimitsNegative (true);
  pass.filter (*input_cloud);

  model::PointCloud point_cloud (input_cloud);

  const float alpha_rad = M_PI/2.0f;
  //const float alpha_rad = 0.0f;
  const float beta_rad = M_PI/2.2f;
  //const float beta_rad = 0.0f;
  const float gamma_rad = M_PI/2.5f;
  //const float gamma_rad = 0.0f;

  model::PointCloud syn_cloud = pertubPointCloud(point_cloud, 
      alpha_rad, beta_rad, gamma_rad);

  visualization::DebugVisualizer::getInstance()
    .visualizePointCloudDiff(point_cloud, syn_cloud);  
  /*
  model::PointCloud cur_sphere = 
    common::SphericalProjection::convertPointCloudCopy(point_cloud);
  model::PointCloud syn_sphere = 
    common::SphericalProjection::convertPointCloudCopy(syn_cloud);
    */

  backend::SphericalCorrelation sph_corr; 
  const int bw = 64;
  auto start = std::chrono::steady_clock::now();
  std::vector<float> f_values = common::SphericalSampler::sampleUniformly(point_cloud, bw);
  std::vector<float> h_values = common::SphericalSampler::sampleUniformly(syn_cloud, bw);
  visualization::DebugVisualizer::getInstance().writeFunctionValuesToFile("F1.txt", f_values);  
  visualization::DebugVisualizer::getInstance().writeFunctionValuesToFile("F2.txt", h_values);  
  std::array<double, 3> zyz = sph_corr.correlateSignals(f_values, h_values, bw);
  auto end = std::chrono::steady_clock::now();

  VLOG(1) << "Done processing point cloud. it took " 
    << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
    << "ms";
  
  model::PointCloud reg_cloud = common::RotationUtils::RotateAroundZYZCopy(
      syn_cloud, zyz[2], zyz[1], zyz[0]);
  visualization::DebugVisualizer::getInstance()
    .visualizePointCloudDiff(point_cloud, reg_cloud);  
}

model::PointCloud Distributor::pertubPointCloud(model::PointCloud &cloud, 
    const float alpha_rad, const float beta_rad, const float gamma_rad) {
  return common::RotationUtils::RotateAroundXYZCopy(
      cloud, alpha_rad, beta_rad, gamma_rad);
}

}
