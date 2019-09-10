#include <packlo/visualization/debug-visualizer.h>
#include <packlo/model/point-cloud.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/impl/pcl_visualizer.hpp>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>
#include <pcl/io/pcd_io.h>

#include <thread>
#include <fstream>

namespace visualization {
  
void DebugVisualizer::writeFunctionValuesToFile(std::string &&file_name,
        const std::vector<float>& function_values) {
  std::ofstream out_file(file_name);
  if (!out_file.is_open()) return;

  for (float value : function_values) {
    out_file << value << "\n";
    out_file << "0.0" << "\n";
  }
  
  out_file.close();
}

void DebugVisualizer::visualizePointCloud(const model::PointCloud& cloud) {

  pcl::visualization::PCLVisualizer::Ptr viewer 
    (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<model::Point_t> (cloud.getRawCloud(), "Point Cloud");
  viewer->setPointCloudRenderingProperties 
    (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Point Cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  using namespace std::chrono_literals;
  while (!viewer->wasStopped ()) {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }
}

void DebugVisualizer::visualizePointCloudDiff(const model::PointCloud& cloud1, 
    const model::PointCloud& cloud2) {
  pcl::visualization::PCLVisualizer::Ptr viewer 
    (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);

  // Add first cloud.
  const model::PointCloud_tPtr cloud1_ptr = cloud1.getRawCloud();
  pcl::visualization::PointCloudColorHandlerCustom<model::Point_t> cloud1_color
    (cloud1_ptr, 0, 255, 0);
  viewer->addPointCloud<model::Point_t> (
      cloud1_ptr, cloud1_color, "cloud1");
  viewer->setPointCloudRenderingProperties 
    (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
  
  // Add second cloud.
  const model::PointCloud_tPtr cloud2_ptr = cloud2.getRawCloud();
  pcl::visualization::PointCloudColorHandlerCustom<model::Point_t> cloud2_color
    (cloud2_ptr, 255, 0, 0);
  viewer->addPointCloud<model::Point_t> (
      cloud2_ptr, cloud2_color, "cloud2");
  viewer->setPointCloudRenderingProperties 
    (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");

  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  using namespace std::chrono_literals;
  while (!viewer->wasStopped ()) {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }
}

void DebugVisualizer::writePcdFile(std::string &&file_name,
    const model::PointCloud &cloud) {
  pcl::io::savePCDFileASCII (file_name, *cloud.getRawCloud());
}

}