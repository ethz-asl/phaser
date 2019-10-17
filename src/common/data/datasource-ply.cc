#include "packlo/common/data/datasource-ply.h"
#include "packlo/common/data/file-system-helper.h"

#include <glog/logging.h>

DEFINE_string(PlyReadDirectory, "", 
    "Defines the directory to read the PLYs from.");

namespace data {

void DatasourcePly::subscribeToPointClouds(
    boost::function<void(const model::PointCloudPtr&)> func) {
  callbacks_.emplace_back(func);
}

void DatasourcePly::startStreaming() {
  std::vector<model::PointCloudPtr> clouds = readPly(FLAGS_PlyReadDirectory); 
  for (model::PointCloudPtr& cloud : clouds) {
    for (auto& callback : callbacks_) {
      callback(cloud);
    }
  }
}

std::vector<model::PointCloudPtr> DatasourcePly::readPly(
    const std::string& directory) {
  std::vector<model::PointCloudPtr> clouds;
  if (directory.empty()) return clouds;
  std::vector<std::string> files;
  FileSystemHelper::readDirectory(directory, &files); 
  if (files.empty()) return clouds;

  for (const std::string& ply : files) {
    model::PointCloudPtr cur_cloud 
      = std::make_shared<model::PointCloud>(directory + ply);
    clouds.emplace_back(std::move(cur_cloud));
  }

  return clouds;
}

} // namespace data
