#include "phaser/model/point-cloud.h"

#include <chrono>
#include <glog/logging.h>
#include <omp.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include "phaser/common/core-gflags.h"
#include "phaser/common/data/file-system-helper.h"
#include "phaser/common/data/ply-helper.h"

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
      ply_directory_(FLAGS_PlyWriteDirectory) {
  squared_voxel_size_ = calcSquaredVoxelSize();
}

PointCloud::PointCloud(common::PointCloud_tPtr cloud)
    : cloud_(cloud),
      info_cloud_(new common::PointCloud_t),
      kd_tree_is_initialized_(false),
      ply_directory_(FLAGS_PlyWriteDirectory) {
  ranges_.resize(cloud_->size());
  squared_voxel_size_ = calcSquaredVoxelSize();
}

PointCloud::PointCloud(const std::string& ply)
    : cloud_(new common::PointCloud_t),
      info_cloud_(new common::PointCloud_t),
      kd_tree_is_initialized_(false),
      ply_directory_(FLAGS_PlyWriteDirectory) {
  readFromFile(ply);
  ranges_.resize(cloud_->size());
  squared_voxel_size_ = calcSquaredVoxelSize();
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
  squared_voxel_size_ = calcSquaredVoxelSize();
}

float PointCloud::calcSquaredVoxelSize() const {
  float voxel_size =
      (std::abs(phaser_core::FLAGS_phaser_core_spatial_low_pass_lower_bound) +
       std::abs(phaser_core::FLAGS_phaser_core_spatial_low_pass_upper_bound)) /
      static_cast<float>(phaser_core::FLAGS_phaser_core_spatial_n_voxels);
  return voxel_size * voxel_size;
}

void PointCloud::initialize_kd_tree() {
  VLOG(1) << "Kd tree is initialized: " << std::boolalpha
          << kd_tree_is_initialized_ << ". for file: " << ply_read_directory_
          << ".";
  if (kd_tree_is_initialized_)
    return;
  kd_tree_.setInputCloud(cloud_);
  kd_tree_is_initialized_ = true;
  VLOG(1) << "Initialized kd tree.";
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
  const uint32_t n_points = query_points.size();
  function_values->resize(n_points);

  // #pragma omp parallel for num_threads(2)
  for (uint32_t i = 0u; i < n_points; ++i) {
    const common::Point_t& query_point = query_points[i];
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
          i, pointIdxNKNSearch, pointNKNSquaredDistance, function_values);
    } else {
      sampleNearestWithoutCloudInfo(
          i, pointIdxNKNSearch, pointNKNSquaredDistance, function_values);
    }
  }
}

void PointCloud::sampleNearestWithoutCloudInfo(
    const uint32_t idx, const std::vector<int>& pointIdxNKNSearch,
    const std::vector<float>& pointNKNSquaredDistance,
    std::vector<FunctionValue>* function_values) const {
  CHECK_NOTNULL(cloud_);
  CHECK_NOTNULL(function_values);
  // Approximate the function value given the neighbors.
  FunctionValue& value = (*function_values)[idx];
  CHECK_GT(FLAGS_sampling_neighbors, 0);
  for (int16_t i = 0u; i < FLAGS_sampling_neighbors; ++i) {
    const float sq_dist = pointNKNSquaredDistance[i];
    if (sq_dist > squared_voxel_size_) {
      continue;
    }
    const int current_idx = pointIdxNKNSearch[i];
    if (current_idx < 0 || current_idx >= cloud_->size()) {
      continue;
    }

    const common::Point_t& point = cloud_->points[current_idx];
    value.addPoint(point);
    value.addRange(ranges_.at(current_idx));
    value.addIntensity(point.intensity);
    if (!reflectivities_.empty())
      value.addReflectivity(reflectivities_.at(current_idx));
    if (!ambient_points_.empty())
      value.addAmbientNoise(ambient_points_.at(current_idx));
  }
}

void PointCloud::sampleNearestWithCloudInfo(
    const uint32_t idx, const std::vector<int>& pointIdxNKNSearch,
    const std::vector<float>& pointNKNSquaredDistance,
    std::vector<FunctionValue>* function_values) const {
  CHECK_NOTNULL(function_values);
  CHECK_NOTNULL(info_cloud_);
  CHECK_NOTNULL(cloud_);
  // Approximate the function value given the neighbors.
  FunctionValue& value = (*function_values)[idx];
  CHECK_GT(FLAGS_sampling_neighbors, 0);
  for (int16_t i = 0u; i < FLAGS_sampling_neighbors; ++i) {
    const float sq_dist = pointNKNSquaredDistance[i];
    if (sq_dist > squared_voxel_size_) {
      continue;
    }
    const int current_idx = pointIdxNKNSearch[i];
    const common::Point_t& point = cloud_->points[current_idx];
    const common::Point_t& info_point = info_cloud_->points[current_idx];
    value.addPoint(point);
    value.addRange(info_point.x);
    value.addIntensity(info_point.y);
  }
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
  cloned_cloud.reflectivities_ = reflectivities_;
  cloned_cloud.ambient_points_ = ambient_points_;
  cloned_cloud.ply_read_directory_ = ply_read_directory_;
  return cloned_cloud;
}

void PointCloud::setRange(const double range, const uint32_t i) {
  CHECK_LT(i, ranges_.size());
  ranges_.at(i) = range;
}

double PointCloud::rangeAt(const uint32_t i) const {
  CHECK_LT(i, ranges_.size());
  double range = ranges_.at(i);
  if (range == 0) {
    return calcRangeAt(i);
  }
  return range;
}

double PointCloud::calcRangeAt(const uint32_t i) const {
  const common::Point_t& p = pointAt(i);
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

double PointCloud::getReflectivity(const uint32_t i) const {
  CHECK_LT(i, reflectivities_.size());
  return reflectivities_.at(i);
}

double PointCloud::getAmbientNoise(const uint32_t i) const {
  CHECK_LT(i, ambient_points_.size());
  return ambient_points_.at(i);
}

bool PointCloud::hasReflectivityPoints() const {
  return !reflectivities_.empty();
}

bool PointCloud::hasAmbientNoisePoints() const {
  return !ambient_points_.empty();
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
  data::PlyHelper ply_helper;
  model::PlyPointCloud ply_cloud = ply_helper.readPlyFromFile(ply);
  parsePlyPointCloud(std::move(ply_cloud));
  VLOG(2) << "Cloud size: " << cloud_->size();
  initialize_kd_tree();
}

std::string PointCloud::getPlyReadDirectory() const noexcept {
  return ply_read_directory_;
}

void PointCloud::parsePlyPointCloud(PlyPointCloud&& ply_point_cloud) {
  CHECK_NOTNULL(cloud_);
  const std::vector<double>& xyz = ply_point_cloud.getXYZPoints();
  const std::vector<double>& intensities = ply_point_cloud.getIntentsities();
  const uint32_t n_points = xyz.size();
  const uint32_t n_intensities = intensities.size();
  CHECK_GT(n_points, 0u);
  uint32_t k = 0u;
  for (uint32_t i = 0u; i < n_points && k < n_intensities; i += 3u) {
    common::Point_t p;
    p.x = xyz[i];
    p.y = xyz[i + 1u];
    p.z = xyz[i + 2u];
    p.intensity = intensities[k];
    cloud_->points.push_back(p);
    ++k;
  }
  ambient_points_ = ply_point_cloud.getAmbientPoints();
  ranges_ = ply_point_cloud.getRange();
  reflectivities_ = ply_point_cloud.getReflectivities();
}

}  // namespace model
