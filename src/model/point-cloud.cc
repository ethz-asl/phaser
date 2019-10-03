#include "packlo/model/point-cloud.h"

#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <pcl/io/ply_io.h>

#include <boost/filesystem.hpp>

#include <glog/logging.h>
#include <chrono>

DEFINE_string(PlyWriteDirectory, "", 
		"Defines the directory to store the point clouds.");
DEFINE_string(PlyPrefix, "cloud", 
		"Defines the prefix name for the PLY.");

namespace model {

PointCloud::PointCloud(common::PointCloud_tPtr cloud) 
  : cloud_(cloud), kd_tree_is_initialized_(false) {
}

void PointCloud::initialize_kd_tree() {
  kd_tree_.setInputCloud(cloud_);
	kd_tree_is_initialized_ = true;
}

common::PointCloud_t::iterator PointCloud::begin() {
  return cloud_->begin();
}

common::PointCloud_t::iterator PointCloud::end() {
  return cloud_->end();
}

void PointCloud::getNearestPoints(
		const std::vector<common::Point_t> &query_points, 
		std::vector<FunctionValue>* function_values) const {
	CHECK(kd_tree_is_initialized_);
  std::vector<int> pointIdxNKNSearch(kNeighbors);
  std::vector<float> pointNKNSquaredDistance(kNeighbors);

  for (const common::Point_t& query_point : query_points) {
    // First, find the closest points. 
    const int kd_tree_res = kd_tree_.nearestKSearch (query_point, kNeighbors,
        pointIdxNKNSearch, pointNKNSquaredDistance);
    if (kd_tree_res <= 0) { 
      VLOG(2) << "Unable to find nearest neighbor. Skipping point.";
      continue;
    }
    
    // Approximate the function value given the neighbors. 
		FunctionValue value;
    for (size_t i = 0u; i < kNeighbors; ++i) {
      const int current_idx = pointIdxNKNSearch[i];
      const common::Point_t& point = cloud_->points[current_idx]; 
      const double dist = std::sqrt(point.x * point.x + 
																		point.y * point.y + 
																		point.z * point.z);
			value.addPoint(point);
			value.addRange(dist);
			value.addIntensity(point.intensity);
			value.addInterpolation(0.40f * point.intensity + 0.60f * dist);
    }
    function_values->emplace_back(std::move(value));
  }
}

void PointCloud::transformPointCloud(const Eigen::Matrix4f &T) {
  pcl::transformPointCloud (*cloud_, *cloud_, T);
}

void PointCloud::transformPointCloudCopy(
		const Eigen::Matrix4f& T, PointCloud& copy) {
  pcl::transformPointCloud (*cloud_, *copy.cloud_, T);
}

common::PointCloud_tPtr PointCloud::getRawCloud() const {
  return cloud_;
}

common::Point_t& PointCloud::pointAt(const std::size_t idx) {
  return cloud_->points[idx];  
}

const common::Point_t& PointCloud::pointAt(const std::size_t idx) const {
  return cloud_->points[idx];  
}

std::size_t PointCloud::size() const {
  return cloud_->points.size();
}

PointCloud PointCloud::clone() const {
  common::PointCloud_tPtr cloned (new common::PointCloud_t);  
  pcl::copyPointCloud(*cloud_, *cloned);
  return PointCloud(cloned);
}

void PointCloud::writeToFile() {
	CHECK(!FLAGS_PlyWriteDirectory.empty());
	pcl::PLYWriter writer;
	std::vector<std::string> files;
	read_directory(FLAGS_PlyWriteDirectory, &files);	
	std::string file_name = FLAGS_PlyWriteDirectory + FLAGS_PlyPrefix 
			 + std::to_string(files.size() + 1) + ".ply";

	writer.write(file_name,	*cloud_);
}

void PointCloud::read_directory(const std::string& directory, 
		std::vector<std::string>* files) const {
	boost::filesystem::path p(directory);
	boost::filesystem::directory_iterator start(p);
	boost::filesystem::directory_iterator end;
	std::transform(start, end, std::back_inserter(*files), 
			[] (const boost::filesystem::directory_entry& entry) {
        return entry.path().leaf().string();
    });
}


}
