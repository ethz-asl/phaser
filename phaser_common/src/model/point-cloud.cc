#include "phaser/model/point-cloud.h"
#include "phaser/common/data/file-system-helper.h"

#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <chrono>
#include <glog/logging.h>

DEFINE_string(
    PlyWriteDirectory, "", "Defines the directory to store the point clouds.");
DEFINE_string(PlyPrefix, "cloud", "Defines the prefix name for the PLY.");
DEFINE_int32(
    sampling_neighbors, 1, "Defines the number of neighbors for the sampling.");
DEFINE_double(
    neighbor_max_distance, 5,
    "Defines the maximum allowed distance to neighbors.");

namespace model {

PointCloud::PointCloud()
    : cloud_(new common::PointCloud_t),
      info_cloud_(new common::PointCloud_t),
      kd_tree_is_initialized_(false),
      ply_directory_(FLAGS_PlyWriteDirectory) {}

PointCloud::PointCloud(common::PointCloud_tPtr cloud)
    : cloud_(cloud),
      info_cloud_(new common::PointCloud_t),
      kd_tree_is_initialized_(false),
      ply_directory_(FLAGS_PlyWriteDirectory) {
  ranges_.resize(cloud_->size());
}

PointCloud::PointCloud(const std::string& ply)
    : cloud_(new common::PointCloud_t),
      info_cloud_(new common::PointCloud_t),
      kd_tree_is_initialized_(false),
      ply_directory_(FLAGS_PlyWriteDirectory) {
  readFromFile(ply);
  ranges_.resize(cloud_->size());
}

PointCloud::PointCloud(const std::vector<common::Point_t>& points)
    : cloud_(new common::PointCloud_t),
      info_cloud_(new common::PointCloud_t),
      kd_tree_is_initialized_(false),
      ply_directory_(FLAGS_PlyWriteDirectory) {
  for (const common::Point_t& point : points) {
    cloud_->push_back(point);
  }
  ranges_.resize(cloud_->size());
}

void PointCloud::initialize_kd_tree() {
  if (kd_tree_is_initialized_)
    return;
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
  CHECK_NOTNULL(cloud_);
  CHECK_NOTNULL(function_values);
  std::vector<int> pointIdxNKNSearch(FLAGS_sampling_neighbors);
  std::vector<float> pointNKNSquaredDistance(FLAGS_sampling_neighbors);

  const bool info_cloud_is_available = hasInfoCloud();
  VLOG(2) << "Sampling using info cloud: " << std::boolalpha
          << info_cloud_is_available << ".";
  for (const common::Point_t& query_point : query_points) {
    // First, find the closest points.
    const int kd_tree_res = kd_tree_.nearestKSearch(
        query_point, FLAGS_sampling_neighbors, pointIdxNKNSearch,
        pointNKNSquaredDistance);
    if (kd_tree_res <= 0) {
      VLOG(2) << "Unable to find nearest neighbor. Skipping point.";
      continue;
    }
    if (info_cloud_is_available) {
      sampleNearestWithCloudInfo(
          pointIdxNKNSearch, pointNKNSquaredDistance, function_values);
    } else {
      sampleNearestWithoutCloudInfo(
          pointIdxNKNSearch, pointNKNSquaredDistance, function_values);
    }
  }
}

void PointCloud::sampleNearestWithoutCloudInfo(
    const std::vector<int>& pointIdxNKNSearch,
    const std::vector<float>& pointNKNSquaredDistance,
    std::vector<FunctionValue>* function_values) const {
  CHECK_NOTNULL(cloud_);
  CHECK_NOTNULL(function_values);
  // Approximate the function value given the neighbors.
  FunctionValue value;
  CHECK_GT(FLAGS_sampling_neighbors, 0);
  for (int16_t i = 0u; i < FLAGS_sampling_neighbors; ++i) {
    const int current_idx = pointIdxNKNSearch[i];
    const common::Point_t& point = cloud_->points[current_idx];
    value.addPoint(point);
    value.addRange(ranges_.at(current_idx));
    value.addIntensity(point.intensity);
  }
  function_values->emplace_back(std::move(value));
}

void PointCloud::sampleNearestWithCloudInfo(
    const std::vector<int>& pointIdxNKNSearch,
    const std::vector<float>& pointNKNSquaredDistance,
    std::vector<FunctionValue>* function_values) const {
  CHECK_NOTNULL(function_values);
  CHECK_NOTNULL(info_cloud_);
  CHECK_NOTNULL(cloud_);
  // Approximate the function value given the neighbors.
  FunctionValue value;
  CHECK_GT(FLAGS_sampling_neighbors, 0);
  for (int16_t i = 0u; i < FLAGS_sampling_neighbors; ++i) {
    const int current_idx = pointIdxNKNSearch[i];
    const common::Point_t& point = cloud_->points[current_idx];
    const common::Point_t& info_point = info_cloud_->points[current_idx];
    value.addPoint(point);
    value.addRange(info_point.x);
    value.addIntensity(info_point.y);
  }
  function_values->emplace_back(std::move(value));
}

void PointCloud::transformPointCloud(const Eigen::Matrix4f& T) {
  pcl::transformPointCloud(*cloud_, *cloud_, T);
}

void PointCloud::transformPointCloudCopy(
    const Eigen::Matrix4f& T, PointCloud* copy) const {
  pcl::transformPointCloud(*cloud_, *(*copy).cloud_, T);

  // Set the ranges to zero since they are outdated now.
  copy->ranges_.clear();
  copy->ranges_.resize(cloud_->size());
}

common::PointCloud_tPtr PointCloud::getRawCloud() const {
  return cloud_;
}

common::PointCloud_tPtr& PointCloud::getRawCloud() {
  return cloud_;
}

common::PointCloud_tPtr PointCloud::getRawInfoCloud() const {
  return info_cloud_;
}

common::PointCloud_tPtr& PointCloud::getRawInfoCloud() {
  return info_cloud_;
}

bool PointCloud::hasInfoCloud() const {
  return info_cloud_ != nullptr && info_cloud_->size() > 0;
}

common::Point_t& PointCloud::pointAt(const std::size_t idx) {
  CHECK_NOTNULL(cloud_);
  CHECK(idx < cloud_->size());
  return cloud_->points[idx];
}

const common::Point_t& PointCloud::pointAt(const std::size_t idx) const {
  CHECK_NOTNULL(cloud_);
  CHECK(idx < cloud_->size());
  return cloud_->points[idx];
}

common::Point_t& PointCloud::infoPointAt(const std::size_t idx) {
  CHECK_NOTNULL(cloud_);
  CHECK(idx < info_cloud_->size());
  return info_cloud_->points[idx];
}

const common::Point_t& PointCloud::infoPointAt(const std::size_t idx) const {
  CHECK_NOTNULL(cloud_);
  CHECK(idx < info_cloud_->size());
  return info_cloud_->points[idx];
}

std::size_t PointCloud::size() const {
  CHECK_NOTNULL(cloud_);
  return cloud_->points.size();
}

PointCloud PointCloud::clone() const {
  PointCloud cloned_cloud;
  CHECK_NOTNULL(cloud_);
  CHECK_NOTNULL(cloned_cloud.cloud_);
  CHECK_NOTNULL(info_cloud_);
  CHECK_NOTNULL(cloned_cloud.info_cloud_);
  pcl::copyPointCloud(*cloud_, *cloned_cloud.cloud_);
  pcl::copyPointCloud(*info_cloud_, *cloned_cloud.info_cloud_);
  cloned_cloud.ranges_ = ranges_;
  return cloned_cloud;
}

void PointCloud::setRange(const double range, const uint32_t i) {
  CHECK_LT(i, ranges_.size());
  ranges_.at(i) = range;
}

double PointCloud::getRange(const uint32_t i) const {
  CHECK_LT(i, ranges_.size());
  return ranges_.at(i);
}

void PointCloud::writeToFile(std::string&& directory) {
  if (directory.empty())
    directory = ply_directory_;
  CHECK(!directory.empty());
  pcl::PLYWriter writer;
  std::vector<std::string> files;
  data::FileSystemHelper::readDirectory(directory, &files);
  char buffer[50];
  snprintf(
      buffer, 50, "%s%.3lu.ply", FLAGS_PlyPrefix.c_str(), files.size() + 1);
  const std::string full_name = directory + std::string(buffer);

  VLOG(2) << "Writing PLY file to: " << full_name;
  writer.write(full_name, *cloud_);
}

void PointCloud::readFromFile(const std::string& ply) {
  CHECK(!ply.empty());
  CHECK_NOTNULL(cloud_);
  VLOG(2) << "Reading PLY file from: " << ply;
  ply_read_directory_ = ply;
  pcl::PLYReader reader;
  reader.read(ply, *cloud_);
  VLOG(2) << "Cloud size: " << cloud_->size();
  // convertInputPointCloud(cloud);
}

std::string PointCloud::getPlyReadDirectory() const noexcept {
  return ply_read_directory_;
}

}  // namespace model
