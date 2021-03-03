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
  explicit DatasourceFeatures(const std::string& datasource_folder);
  virtual void subscribeToFeatures(
      boost::function<void(common::SphericalFeature*)> func);
  virtual void startStreaming();

 private:
  std::vector<boost::function<void(common::SphericalFeature*)>> callbacks_;
  bool readAllFeatures();
  void readFeature(const std::string& directory, PhaserFeature* feature);
  std::vector<double> parseLine(std::ifstream* in_file);
  const std::string& datasource_folder_;
};

using DatasourceFeaturesPtr = std::unique_ptr<DatasourceFeatures>;

}  // namespace data

#endif  // PHASER_COMMON_DATA_DATASOURCE_FEATURES_H_
