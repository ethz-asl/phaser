#include <packlo/model/point-cloud.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>

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

  /*
  VLOG(1) << "======================================";
  // TEST ---------------------
  for (std::size_t i = 0u; i < 350; ++i) {
    
    auto qpoint = query_points[i];
    const int kd_tree_res = kd_tree.nearestKSearch (qpoint, kNeighbors,
        pointIdxNKNSearch, pointNKNSquaredDistance);
    if (kd_tree_res <= 0) continue;

    float interpolation = 0.0f;
    VLOG(1) << "query point [" << i << "] : " << qpoint.x << " " << qpoint.y << " " << qpoint.z;
    const std::size_t n_neighbors = pointIdxNKNSearch.size();
    for (size_t i = 0u; i < n_neighbors; ++i) {
      int current_idx = pointIdxNKNSearch[i];
      const model::Point_t& point = cloud_->points[current_idx]; 
      double dist_xy = std::sqrt(point.x * point.x + point.y*point.y);
      double dist = std::sqrt(dist_xy * dist_xy + point.z*point.z);
      interpolation += 0.33f * cloud_->points[current_idx].intensity
          + 0.67f * dist;
      VLOG(1) << "point: " << point.x << " " << point.y << " " << point.z;
      VLOG(1) << "interpolation: " << interpolation;
    }
  }

  // ------------------------------
  
  */


  for (const Point_t& query_point : query_points) {
    // First, find the closest points. 
    const int kd_tree_res = kd_tree.nearestKSearch (query_point, kNeighbors,
        pointIdxNKNSearch, pointNKNSquaredDistance);
    if (kd_tree_res <= 0) { 
      VLOG(1) << "kd tree is 0";
      return function_values;
    }
    
    // Approximate the function value given the neighbors. 
    float interpolation = 0.0f;
    const std::size_t n_neighbors = pointIdxNKNSearch.size();
    for (size_t i = 0u; i < n_neighbors; ++i) {
      int current_idx = pointIdxNKNSearch[i];
      const model::Point_t& point = cloud_->points[current_idx]; 
      double dist_xy = std::sqrt(point.x * point.x + point.y*point.y);
      double dist = std::sqrt(dist_xy * dist_xy + point.z*point.z);
      interpolation += 0.33f * cloud_->points[current_idx].intensity
          + 0.67f * dist;
      //interpolation += dist;
    }
    function_values.emplace_back(interpolation / n_neighbors);
    //function_values.emplace_back(0.0f);
  }
  return function_values;
}

void PointCloud::transformPointCloud(const Eigen::Matrix4f &T) {
  pcl::transformPointCloud (*cloud_, *cloud_, T);
}

void PointCloud::transformPointCloudCopy(const Eigen::Matrix4f& T, PointCloud& copy) {
  pcl::transformPointCloud (*cloud_, *copy.cloud_, T);
}

PointCloud_tPtr PointCloud::getRawCloud() const {
  return cloud_;
}

Point_t& PointCloud::pointAt(const std::size_t idx) {
  return cloud_->points[idx];  
}

const Point_t& PointCloud::pointAt(const std::size_t idx) const {
  return cloud_->points[idx];  
}

std::size_t PointCloud::size() const {
  return cloud_->points.size();
}

PointCloud PointCloud::clone() const {
  PointCloud_tPtr cloned (new PointCloud_t);  
  pcl::copyPointCloud(*cloud_, *cloned);
  return PointCloud(cloned);
}

}
