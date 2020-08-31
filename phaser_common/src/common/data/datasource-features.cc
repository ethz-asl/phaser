#include "phaser/common/data/datasource-features.h"
#include "phaser/common/data/file-system-helper.h"

#include <fstream>
#include <sstream>
#include <string>

#include <boost/filesystem.hpp>
#include <glog/logging.h>

DEFINE_string(
    FeaturesReadDirectory, "",
    "Defines the directory to read the features from.");

namespace data {

DatasourceFeatures::DatasourceFeatures()
    : datasource_folder_(FLAGS_FeaturesReadDirectory) {}

void DatasourceFeatures::subscribeToFeatures(
    boost::function<void(const common::SphericalFeature&)> func) {
  callbacks_.emplace_back(func);
}

void DatasourceFeatures::startStreaming() {
  VLOG(1) << "reading features from: " << datasource_folder_;
  PhaserFeatureVec features = readAllFeatures();
  CHECK(!features.empty());
  const uint32_t n_features = features.size();
  VLOG(1) << "Reading features done. number of features: " << n_features;
  for (uint32_t i = 0u; i < n_features; ++i) {
    common::SphericalFeature& feature = features.at(i);
    for (auto& callback : callbacks_) {
      callback(feature);
    }
  }
}

void DatasourceFeatures::setDatasetFolder(std::string&& datasource) {
  datasource_folder_ = datasource;
}

PhaserFeatureVec DatasourceFeatures::readAllFeatures() {
  PhaserFeatureVec features;
  if (datasource_folder_.empty())
    return features;

  const std::string range_feature = datasource_folder_ + "range";
  const std::string intensity_feature = datasource_folder_ + "intensity";
  const std::string visual_feature = datasource_folder_ + "visual";
  const std::string csv = ".csv";

  std::vector<std::string> files;
  FileSystemHelper::readDirectory(datasource_folder_, &files);
  if (files.empty())
    return features;

  const uint32_t n_files = files.size();
  const uint32_t files_per_type = n_files / 3.0;
  for (uint32_t i = 0u; i < files_per_type; ++i) {
    const std::string path_to_range = range_feature + std::to_string(i) + csv;
    const std::string path_to_intensity =
        intensity_feature + std::to_string(i) + csv;
    const std::string path_to_visual = visual_feature + std::to_string(i) + csv;

    common::SphericalFeature feature;
    readFeature(path_to_range, &feature.getRangeFeatures());
    readFeature(path_to_intensity, &feature.getIntensityFeatures());
    readFeature(path_to_visual, &feature.getVisualFeatures());
    features.emplace_back(std::move(feature));
  }
  return features;
}

void DatasourceFeatures::readFeature(
    const std::string& path_to_feature, PhaserFeature* feature) {
  CHECK_NOTNULL(feature);
  std::ifstream infile(path_to_feature);
  CHECK(infile.is_open());
  VLOG(1) << "Reading feature file: " << path_to_feature;
  std::vector<double> line = parseLine(&infile);
  while (!line.empty()) {
    // CHECK_EQ(line.size(), 4);
    feature->insert(feature->end(), line.begin(), line.end());
    line = parseLine(&infile);
  }
  VLOG(2) << "Extracted " << feature->size() << " features.";
}

std::vector<double> DatasourceFeatures::parseLine(std::ifstream* in_file) {
  CHECK_NOTNULL(in_file);
  CHECK(in_file->is_open());
  std::string line;
  std::getline(*in_file, line);
  std::stringstream line_stream(line);

  std::string cell;
  std::vector<double> result;
  while (std::getline(line_stream, cell, ',')) {
    result.emplace_back(std::stod(cell));
  }
  return result;
}

}  // namespace data
