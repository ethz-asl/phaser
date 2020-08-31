#ifndef PHASER_COMMON_DATA_DATASOURCE_FEATURES_H_
#define PHASER_COMMON_DATA_DATASOURCE_FEATURES_H_

#include <memory>
#include <string>
#include <vector>

#include <boost/function.hpp>

#include "phaser/common/spherical-feature.h"

namespace data {

using PhaserFeature = std::vector<double>;
using PhaserFeatureVec = std::vector<common::SphericalFeature>;

class DatasourceFeatures {
 public:
  DatasourceFeatures();
  virtual void subscribeToFeatures(
      boost::function<void(const common::SphericalFeature&)> func);
  virtual void startStreaming();
  void setDatasetFolder(std::string&& datasource);

 private:
  std::vector<boost::function<void(const common::SphericalFeature&)>>
      callbacks_;
  PhaserFeatureVec readAllFeatures();
  void readFeature(const std::string& directory, PhaserFeature* feature);
  std::vector<double> parseLine(std::ifstream* in_file);
  std::string datasource_folder_;
};

using DatasourceFeaturesPtr = std::unique_ptr<DatasourceFeatures>;

}  // namespace data

#endif  // PHASER_COMMON_DATA_DATASOURCE_FEATURES_H_
