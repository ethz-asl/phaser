#include "packlo/model/point-cloud.h"
#include "packlo/common/data/file-system-helper.h"

#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <glog/logging.h>
#include <chrono>

DEFINE_string(
    PlyWriteDirectory, "", "Defines the directory to store the point clouds.");
DEFINE_string(PlyPrefix, "cloud", "Defines the prefix name for the PLY.");
DEFINE_int32(
    sampling_neighbors, 1, "Defines the number of neighbors for the sampling.");

namespace model {

PointCloud::PointCloud()
    : cloud_(new common::PointCloud_t),
      cloud_info_(new common::ExtractedPointCloud_t),
      kd_tree_is_initialized_(false),
      ply_directory_(FLAGS_PlyWriteDirectory) {}

PointCloud::PointCloud(common::PointCloud_tPtr cloud)
    : cloud_(cloud),
      cloud_info_(new common::ExtractedPointCloud_t),
      kd_tree_is_initialized_(false),
      ply_directory_(FLAGS_PlyWriteDirectory) {}

PointCloud::PointCloud(common::ExtractedPointCloud_tPtr cloud)
    : cloud_(new common::PointCloud_t),
      cloud_info_(new common::ExtractedPointCloud_t),
      kd_tree_is_initialized_(false),
      ply_directory_(FLAGS_PlyWriteDirectory) {
  convertInputPointCloud(cloud);
}

PointCloud::PointCloud(const std::string& ply)
    : cloud_(new common::PointCloud_t),
      cloud_info_(new common::ExtractedPointCloud_t),
      kd_tree_is_initialized_(false),
      ply_directory_(FLAGS_PlyWriteDirectory) {
  readFromFile(ply);
}

void PointCloud::initialize_kd_tree() {
  if (kd_tree_is_initialized_) return;
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
    const std::vector<common::Point_t>& query_points,
    std::vector<FunctionValue>* function_values) const {
  CHECK(kd_tree_is_initialized_);
  std::vector<int> pointIdxNKNSearch(FLAGS_sampling_neighbors);
  std::vector<float> pointNKNSquaredDistance(FLAGS_sampling_neighbors);

  for (const common::Point_t& query_point : query_points) {
    // First, find the closest points.
    const int kd_tree_res = kd_tree_.nearestKSearch(
        query_point, FLAGS_sampling_neighbors, pointIdxNKNSearch,
        pointNKNSquaredDistance);
    if (kd_tree_res <= 0) {
      VLOG(2) << "Unable to find nearest neighbor. Skipping point.";
      continue;
    }

    // Approximate the function value given the neighbors.
    FunctionValue value;
    for (size_t i = 0u; i < FLAGS_sampling_neighbors; ++i) {
      const int current_idx = pointIdxNKNSearch[i];
      const common::Point_t& point = cloud_->points[current_idx];
      const common::ExtractedPoint_t point_info =
          cloud_info_->points[current_idx];

      const double dist =
          std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
      value.addPoint(point);
      value.addRange(dist);
      value.addIntensity(point_info.intensity);
      value.addSemanticClass(point_info.semantic);
      value.addSemanticClass(point_info.instance);
      value.addInterpolation(point_info.semantic);
    }
    function_values->emplace_back(std::move(value));
  }
}

void PointCloud::transformPointCloud(const Eigen::Matrix4f &T) {
  pcl::transformPointCloud(*cloud_, *cloud_, T);
}

void PointCloud::transformPointCloudCopy(
    const Eigen::Matrix4f& T, PointCloud* copy) const {
  pcl::transformPointCloud(*cloud_, *(*copy).cloud_, T);
  pcl::copyPointCloud(*cloud_info_, *(*copy).cloud_info_);
}

common::PointCloud_tPtr PointCloud::getRawCloud() const {
  return cloud_;
}

common::ExtractedPointCloud_tPtr PointCloud::getRawInfoCloud() const {
  return cloud_info_;
}

common::Point_t& PointCloud::pointAt(const std::size_t idx) {
  CHECK_NOTNULL(cloud_);
  return cloud_->points[idx];
}

const common::Point_t& PointCloud::pointAt(const std::size_t idx) const {
  CHECK_NOTNULL(cloud_);
  return cloud_->points[idx];
}

common::ExtractedPoint_t& PointCloud::pointInfoAt(const std::size_t idx) {
  CHECK_NOTNULL(cloud_info_);
  return cloud_info_->points[idx];
}

const common::ExtractedPoint_t& PointCloud::pointInfoAt(
    const std::size_t idx) const {
  CHECK_NOTNULL(cloud_info_);
  return cloud_info_->points[idx];
}

std::size_t PointCloud::size() const {
  CHECK_NOTNULL(cloud_);
  return cloud_->points.size();
}

std::size_t PointCloud::sizeInfo() const {
  CHECK_NOTNULL(cloud_info_);
  return cloud_info_->points.size();
}

void PointCloud::convertInputPointCloud(
    common::ExtractedPointCloud_tPtr cloud) {
  CHECK_NOTNULL(cloud_info_);
  CHECK_NOTNULL(cloud_);
  pcl::copyPointCloud(*cloud, *cloud_info_);
  pcl::copyPointCloud(*cloud, *cloud_);
  /*
  const uint16_t n_points = cloud->size();
  for (uint16_t i = 0; i < n_points; ++i) {
    const common::ExtractedPoint_t& p = cloud->points.at(i);
    pcl::PointXYZ raw_point(p.x, p.y, p.z);
    cloud_->push_back(raw_point);
  }*/
}

PointCloud PointCloud::clone() const {
  PointCloud cloned_cloud;
  pcl::copyPointCloud(*cloud_info_, *cloned_cloud.cloud_info_);
  pcl::copyPointCloud(*cloud_, *cloned_cloud.cloud_);
  return cloned_cloud;
}

void PointCloud::writeToFile(std::string&& directory) {
  if (directory.empty()) directory = ply_directory_;
  CHECK(!directory.empty());
  pcl::PLYWriter writer;
  std::vector<std::string> files;
  data::FileSystemHelper::readDirectory(directory, &files);
  std::string file_name =
      directory + FLAGS_PlyPrefix + std::to_string(files.size() + 1) + ".ply";

  VLOG(2) << "Writing PLY file to: " << file_name;
  writer.write(file_name, *cloud_info_);
}

void PointCloud::updateInfo(const pcl::IndicesConstPtr indices) {
  // Perform a manual deep copy of the elements,
  // since pcl::ExtractIndices does not work off the shelf
  // with custom point types.
  common::ExtractedPointCloud_tPtr cloud_info_copy(
      new common::ExtractedPointCloud_t);
  VLOG(1) << "size info: " << cloud_info_->points.size();
  VLOG(1) << "size indices: " << indices->size();
  const uint16_t n_points = cloud_info_->points.size();
  for (uint16_t i = 0u; i < n_points; ++i) {
    if (std::find(indices->begin(), indices->end(), i) != indices->end())
      continue;
    cloud_info_copy->points.emplace_back(cloud_info_->points.at(i));
  }
  VLOG(1) << "size: " << cloud_info_copy->points.size();
  cloud_info_ = cloud_info_copy;
}

void PointCloud::updateCloud() {
  pcl::copyPointCloud(*cloud_info_, *cloud_);
}

void PointCloud::readFromFile(const std::string& ply) {
  CHECK(!ply.empty());
  VLOG(2) << "Reading PLY file from: " << ply;
  common::ExtractedPointCloud_tPtr cloud(new common::ExtractedPointCloud_t);
  pcl::PLYReader reader;
  reader.read(ply, *cloud);
  convertInputPointCloud(cloud);
}

}  // namespace model
