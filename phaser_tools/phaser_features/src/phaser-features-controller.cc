#include "phaser/features/phaser-features-controller.h"

#include <sys/stat.h>
#include <sys/types.h>

#include <glog/logging.h>

#include <sstream>

namespace phaser_features {

DEFINE_int32(
    phaser_features_bandwidth, 100,
    "Defines the bandwidth used for the spherical transform.");

PhaserFeatureController::PhaserFeatureController(
    const data::DatasourceFeaturesPtr& ds, const std::string& output_folder)
    : ds_(ds), feature_counter_(0u), output_folder_(output_folder) {
  CHECK_GT(FLAGS_phaser_features_bandwidth, 0);
  ds_->subscribeToFeatures([this](common::SphericalFeature* feature) {
    this->featureCallback(feature);
  });
}

void PhaserFeatureController::featureCallback(
    common::SphericalFeature* feature) {
  // First, get the input feature vector.
  const uint32_t bw_squared =
      FLAGS_phaser_features_bandwidth * FLAGS_phaser_features_bandwidth;
  const std::vector<double>& range_features = feature->getRangeFeatures();
  const std::vector<double>& intensity_features =
      feature->getIntensityFeatures();
  const std::vector<double>& visual_features = feature->getVisualFeatures();

  // Get the output container.
  std::vector<double>& range_transformed = feature->getRangeTransformed();
  std::vector<double>& intensity_transformed =
      feature->getIntensityTransformed();
  std::vector<double>& visual_transformed = feature->getVisualTransformed();

  // Perform the spherical Fourier transform for each feature.
  transformFeatures(range_features, bw_squared, &range_transformed);
  transformFeatures(intensity_features, bw_squared, &intensity_transformed);
  transformFeatures(visual_features, bw_squared, &visual_transformed);
}

void PhaserFeatureController::transformFeatures(
    const std::vector<double>& input_features, const uint32_t bw_squared,
    std::vector<double>* transformed_features) {
  CHECK(!input_features.empty());
  CHECK_NOTNULL(transformed_features);

  double* transformed[2];
  transformed[0] = static_cast<double*>(malloc(sizeof(double) * bw_squared));
  transformed[1] = static_cast<double*>(malloc(sizeof(double) * bw_squared));

  correlation_.performSphericalTransform(input_features, transformed);

  transformed_features->clear();
  transformed_features->resize(bw_squared);
  for (uint32_t i = 0u; i < bw_squared; ++i) {
    transformed_features->at(i) = std::sqrt(
        transformed[0][i] * transformed[0][i] +
        transformed[1][i + 1] * transformed[1][i + 1]);
  }
}

bool PhaserFeatureController::writeTransformedFeaturesToFile(
    const common::SphericalFeature& feature) {
  const std::string path_range = output_folder_ + "range_transformed/";
  const std::string path_intensity = output_folder_ + "intensity_transformed/";
  const std::string path_visual = output_folder_ + "visual_transformed/";
  const std::string csv = ".csv";
  if (!pathExists(path_range)) {
    return false;
  }
  if (!pathExists(path_intensity)) {
    return false;
  }
  if (!pathExists(path_visual)) {
    return false;
  }

  std::ofstream out_file("");

  ++feature_counter_;
  out_file.close();
  return true;
}

bool PhaserFeatureController::writeFeature(
    const std::vector<double>& transformed_features,
    std::ofstream* output_file) {
  CHECK_NOTNULL(output_file);
  const uint32_t linear_size = transformed_features.size();
  const uint32_t double_bw = FLAGS_phaser_features_bandwidth;

  constexpr const char* kDelimiter = ";";
  constexpr const char* kNewLine = "\n";
  for (uint32_t i = 0u; i < double_bw; ++i) {
    for (uint32_t j = 0u; j < double_bw; ++j) {
      const uint32_t linear_idx = i * linear_size + j;
      CHECK_LT(linear_idx, linear_size);
      if (j != double_bw - 1)
        (*output_file) << transformed_features[linear_idx] << kDelimiter;
      else
        (*output_file) << transformed_features[linear_idx];
    }
    (*output_file) << kNewLine;
  }

  return true;
}

bool PhaserFeatureController::pathExists(const std::string& path) {
  struct stat file_status;
  if (stat(path.c_str(), &file_status) == 0 &&
      (file_status.st_mode & S_IFDIR)) {
    return true;
  }
  return false;
}

}  // namespace phaser_features
