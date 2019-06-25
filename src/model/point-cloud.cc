#include <packlo/model/point-cloud.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>

#include <glog/logging.h>

namespace model {

PointCloud::PointCloud(PointCloud_tPtr cloud) 
  : cloud_(cloud) {
  
}

PointCloud_t::iterator PointCloud::begin() {
  return cloud_->begin();
}

PointCloud_t::iterator PointCloud::end() {
  return cloud_->end();
}

std::vector<float> PointCloud::getNearestPoints(const std::vector<Point_t> &query_points) {
  pcl::KdTreeFLANN<Point_t> kd_tree; 
  kd_tree.setInputCloud(cloud_);

  std::vector<int> pointIdxNKNSearch(kNeighbors);
  std::vector<float> pointNKNSquaredDistance(kNeighbors);
  std::vector<float> function_values;

  for (Point_t query_point : query_points) {
    // First, find the closest points. 
    const int kd_tree_res = kd_tree.nearestKSearch (query_point, kNeighbors,
        pointIdxNKNSearch, pointNKNSquaredDistance);
    if (kd_tree_res <= 0) return function_values;
    
    // Approximate the function value given the neighbors. 
    float interpolation = 0.0f;
    const std::size_t n_neighbors = pointIdxNKNSearch.size();
    for (size_t i = 0; i < n_neighbors; ++i) {
      int current_idx = pointIdxNKNSearch[i];
      interpolation += 0.33f * cloud_->points[current_idx].intensity
          + 0.67f * cloud_->points[current_idx].z;
    }
    function_values.emplace_back(interpolation / n_neighbors);
  }
  return function_values;
}

void PointCloud::transformPointCloud(const Eigen::Matrix4f &T) {
  pcl::transformPointCloud (*cloud_, *cloud_, T);
}

void PointCloud::transformPointCloudCopy(const Eigen::Matrix4f& T, PointCloud& copy) {
  pcl::transformPointCloud (*cloud_, *copy.cloud_, T);
}

}
