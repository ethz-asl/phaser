#include "phaser/common/data/datasource-ply.h"

#include <boost/filesystem.hpp>
#include <glog/logging.h>

#include "phaser/common/data/file-system-helper.h"

DEFINE_string(
    PlyReadDirectory, "", "Defines the directory to read the PLYs from.");

namespace data {

DatasourcePly::DatasourcePly() : datasource_folder_(FLAGS_PlyReadDirectory) {}

void DatasourcePly::subscribeToPointClouds(
    boost::function<void(const model::PointCloudPtr&)> func) {
  callbacks_.emplace_back(func);
}

void DatasourcePly::startStreaming(const uint32_t number_of_clouds) {
  VLOG(1) << "reading ply from: " << datasource_folder_;
  std::vector<model::PointCloudPtr> clouds =
      readPly(datasource_folder_, number_of_clouds);
  VLOG(1) << "reading ply done. number of clouds: " << clouds.size();
  CHECK(!clouds.empty());
  const uint32_t n_clouds = clouds.size();
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
    const std::string& directory, const uint32_t max_n_clouds) {
  std::vector<model::PointCloudPtr> clouds;
  if (directory.empty())
    return clouds;
  std::vector<std::string> files;
  FileSystemHelper::readDirectory(directory, &files);
  if (files.empty())
    return clouds;

  uint32_t file_counter = 0u;
  for (const std::string& ply : files) {
    if (max_n_clouds > 0 && file_counter > max_n_clouds)
      break;
    const std::string& path_to_ply = directory + ply;
    boost::filesystem::path p(path_to_ply);
    if (p.extension() != ".ply")
      continue;
    model::PointCloudPtr cur_cloud =
        std::make_shared<model::PointCloud>(path_to_ply);
    clouds.emplace_back(std::move(cur_cloud));
    ++file_counter;
  }

  return clouds;
}

}  // namespace data
