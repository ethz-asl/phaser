#include <packlo/model/point-cloud.h>

#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>

#include <glog/logging.h>
#include <chrono>

namespace model {

PointCloud::PointCloud(PointCloud_tPtr cloud) 
  : cloud_(cloud), kd_tree_is_initialized_(false) {
}

void PointCloud::initialize_kd_tree() {
  kd_tree_.setInputCloud(cloud_);
	kd_tree_is_initialized_ = true;
}

PointCloud_t::iterator PointCloud::begin() {
  return cloud_->begin();
}

PointCloud_t::iterator PointCloud::end() {
  return cloud_->end();
}

void PointCloud::getNearestPoints(
		const std::vector<Point_t> &query_points, 
		std::vector<float>* function_values) const {
	CHECK(kd_tree_is_initialized_);
  std::vector<int> pointIdxNKNSearch(kNeighbors);
  std::vector<float> pointNKNSquaredDistance(kNeighbors);

  for (const Point_t& query_point : query_points) {
    // First, find the closest points. 
    const int kd_tree_res = kd_tree_.nearestKSearch (query_point, kNeighbors,
        pointIdxNKNSearch, pointNKNSquaredDistance);
    if (kd_tree_res <= 0) { 
      VLOG(2) << "Unable to find nearest neighbor. Skipping point.";
      continue;
    }
    
    // Approximate the function value given the neighbors. 
    float interpolation = 0.0f;
    for (size_t i = 0u; i < kNeighbors; ++i) {
      int current_idx = pointIdxNKNSearch[i];
      const model::Point_t& point = cloud_->points[current_idx]; 
      double dist_xy = std::sqrt(point.x * point.x + point.y*point.y);
      double dist = std::sqrt(dist_xy * dist_xy + point.z*point.z);
      interpolation += 0.33f * cloud_->points[current_idx].intensity
          + 0.67f * dist;
    }
    function_values->emplace_back(interpolation / kNeighbors);
  }
}

void PointCloud::transformPointCloud(const Eigen::Matrix4f &T) {
  pcl::transformPointCloud (*cloud_, *cloud_, T);
}

void PointCloud::transformPointCloudCopy(
		const Eigen::Matrix4f& T, PointCloud& copy) {
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
