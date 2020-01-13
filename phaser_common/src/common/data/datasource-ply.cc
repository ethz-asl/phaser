#include "packlo/common/data/datasource-ply.h"
#include "packlo/common/data/file-system-helper.h"

#include <boost/filesystem.hpp>
#include <glog/logging.h>

DEFINE_string(
    PlyReadDirectory, "", "Defines the directory to read the PLYs from.");

namespace data {

DatasourcePly::DatasourcePly() : datasource_folder_(FLAGS_PlyReadDirectory) {}

void DatasourcePly::subscribeToPointClouds(
    boost::function<void(const model::PointCloudPtr&)> func) {
  callbacks_.emplace_back(func);
}

void DatasourcePly::startStreaming(const uint32_t number_of_clouds) {
  VLOG(1) << "path: " << boost::filesystem::current_path();
  VLOG(1) << "reading ply from: " << datasource_folder_;
  std::vector<model::PointCloudPtr> clouds = readPly(datasource_folder_);
  VLOG(1) << "reading ply done. size: " << clouds.size();
  CHECK(!clouds.empty());
  uint32_t n_clouds = 0;
  if (number_of_clouds == 0)
    n_clouds = clouds.size();
  else
    n_clouds = number_of_clouds;
  for (uint32_t i = 0u; i < n_clouds; ++i) {
    model::PointCloudPtr& cloud = clouds.at(i);
    for (auto& callback : callbacks_) {
      callback(cloud);
    }
  }
}

void DatasourcePly::setDatasetFolder(std::string&& datasource) {
  datasource_folder_ = datasource;
}

std::vector<model::PointCloudPtr> DatasourcePly::readPly(
    const std::string& directory) {
  std::vector<model::PointCloudPtr> clouds;
  if (directory.empty()) return clouds;
  std::vector<std::string> files;
  FileSystemHelper::readDirectory(directory, &files);
  if (files.empty()) return clouds;

  for (const std::string& ply : files) {
    const std::string& path_to_ply = directory + ply;
    boost::filesystem::path p(path_to_ply);
    if (p.extension() != ".ply")
      continue;
    model::PointCloudPtr cur_cloud =
        std::make_shared<model::PointCloud>(path_to_ply);
    clouds.emplace_back(std::move(cur_cloud));
  }

  return clouds;
}

}  // namespace data
