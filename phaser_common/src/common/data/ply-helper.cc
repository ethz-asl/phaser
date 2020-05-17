#include <fstream>

#include "tinyply/tinyply.h"

#include "phaser/common/data/ply-helper.h"

namespace data {

model::PlyPointCloud PlyHelper::readPlyFromFile(const std::string& filename) {
  std::ifstream in_stream(filename);
  if (!stream_ply.is_open()) {
    LOG(ERROR) << "Unable to open ply file: " << filename;
  }
  model::PlyPointCloud ply_cloud;
  tinyply::PlyFile ply_file(stream_ply);
  const int xyz_point_count = ply_file.request_properties_from_element(
      "vertex", {"x", "y", "z"}, ply_cloud.getXYZPoints());
  CHECK_GT(xyz_point_count, 0);

  const int intensitiy_point_count = ply_file.request_properties_from_element(
      "vertex", {"intensity"}, ply_cloud.getIntentsities());
  CHECK_GT(intensity_point_count, 0);

  const int refl_point_count = ply_file.request_properties_from_element(
      "vertex", {"reflectivity"}, ply_cloud.getReflectivities());
  LOG_IF(WARNING) << "No reflectivity channel found.";

  const int ambient_point_count = ply_file.request_properties_from_element(
      "vertex", {"noise"}, ply_cloud.getAmbientPoints());
  LOG_IF(WARNING) << "No ambient channel found.";

  const int range_point_count = ply_file.request_properties_from_element(
      "vertex", {"range"}, ply_cloud.getRange());
  LOG_IF(WARNING) << "No range channel found.";

  return ply_cloud;
}

}  // namespace data
