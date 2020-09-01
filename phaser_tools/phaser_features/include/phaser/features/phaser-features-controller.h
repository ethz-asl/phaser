#ifndef PHASER_FEATURES_PHASER_FEATURES_CONTROLLER_H_
#define PHASER_FEATURES_PHASER_FEATURES_CONTROLLER_H_

#include <fstream>
#include <string>
#include <vector>

#include "phaser/backend/correlation/spherical-correlation.h"
#include "phaser/common/data/datasource-features.h"

namespace phaser_features {

class PhaserFeatureController {
 public:
  explicit PhaserFeatureController(
      const data::DatasourceFeaturesPtr& ds, const std::string& output_folder);

 private:
  void featureCallback(common::SphericalFeature* feature);
  void transformFeatures(
      const std::vector<double>& input_feature, const uint32_t bw_squared,
      std::vector<double>* transformed_features);
  bool writeTransformedFeaturesToFile(const common::SphericalFeature& feature);
  bool writeFeature(
      const std::vector<double>& transformed_features,
      std::ofstream* output_file);
  bool pathExists(const std::string& path);

  const data::DatasourceFeaturesPtr& ds_;
  phaser_core::SphericalCorrelation correlation_;
  uint32_t feature_counter_;
  const std::string& output_folder_;
};

}  // namespace phaser_features

#endif  // PHASER_FEATURES_PHASER_FEATURES_CONTROLLER_H_
