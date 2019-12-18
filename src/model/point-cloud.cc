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
      kd_tree_is_initialized_(false),
      ply_directory_(FLAGS_PlyWriteDirectory) {}

PointCloud::PointCloud(common::PointCloud_tPtr cloud)
    : cloud_(cloud),
      kd_tree_is_initialized_(false),
      ply_directory_(FLAGS_PlyWriteDirectory) {}

PointCloud::PointCloud(const std::string& ply)
    : cloud_(new common::PointCloud_t),
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
    CHECK_GT(FLAGS_sampling_neighbors, 0);
    for (int16_t i = 0u; i < FLAGS_sampling_neighbors; ++i) {
      const int current_idx = pointIdxNKNSearch[i];
      const common::Point_t& point = cloud_->points[current_idx];

      const double dist =
          std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
      value.addPoint(point);
      value.addRange(dist);
      value.addIntensity(point.intensity);
      value.addInterpolation(0.60f * point.intensity + 0.40f * dist);
      // value.addInterpolation(point.intensity);
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
}

common::PointCloud_tPtr PointCloud::getRawCloud() const {
  return cloud_;
}

common::Point_t& PointCloud::pointAt(const std::size_t idx) {
  CHECK_NOTNULL(cloud_);
  return cloud_->points[idx];
}

const common::Point_t& PointCloud::pointAt(const std::size_t idx) const {
  CHECK_NOTNULL(cloud_);
  return cloud_->points[idx];
}

std::size_t PointCloud::size() const {
  CHECK_NOTNULL(cloud_);
  return cloud_->points.size();
}


PointCloud PointCloud::clone() const {
  PointCloud cloned_cloud;
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
  writer.write(file_name, *cloud_);
}

void PointCloud::readFromFile(const std::string& ply) {
  CHECK(!ply.empty());
  VLOG(2) << "Reading PLY file from: " << ply;
  // common::ExtractedPointCloud_tPtr cloud(new common::ExtractedPointCloud_t);
  pcl::PLYReader reader;
  reader.read(ply, *cloud_);
  VLOG(2) << "Cloud size: " << cloud_->size();
  // convertInputPointCloud(cloud);
}

}  // namespace model
