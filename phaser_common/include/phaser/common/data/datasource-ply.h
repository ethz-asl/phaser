#pragma once

#include <memory>
#include <vector>

#include "phaser/common/data/base-datasource.h"
#include "phaser/model/point-cloud.h"

namespace data {

class DatasourcePly : public BaseDatasource {
 public:
  DatasourcePly();
  virtual void subscribeToPointClouds(
      boost::function<void(const model::PointCloudPtr&)> func) override;
  virtual void startStreaming(const uint32_t number_of_clouds = 0) override;
  void setDatasetFolder(std::string&& datasource);

 private:
  std::vector<model::PointCloudPtr> readPly(
      const std::string& directory, const uint32_t max_n_clouds);
  std::string datasource_folder_;
};

using DatasourcePlyPtr = std::unique_ptr<DatasourcePly>;

}  // namespace data
