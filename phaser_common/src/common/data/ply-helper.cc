#include "phaser/common/data/ply-helper.h"

#include <fstream>
#include <glog/logging.h>

#include "tinyply/tinyply.h"

DEFINE_string(
    phaser_ply_intensity_str, "intensity",
    "Defines the name of the intensity field in the PLY.");

DEFINE_string(
    phaser_ply_reflectivity_str, "reflectivity",
    "Defines the name of the reflectivity field in the PLY.");

DEFINE_string(
    phaser_ply_ambient_str, "noise",
    "Defines the name of the ambient field in the PLY.");

DEFINE_string(
    phaser_ply_range_str, "range",
    "Defines the name of the range field in the PLY.");

namespace data {

model::PlyPointCloud PlyHelper::readPlyFromFile(const std::string& filename) {
  std::ifstream in_stream(filename);
  if (!in_stream.is_open()) {
    LOG(ERROR) << "Unable to open ply file: " << filename;
  }
  model::PlyPointCloud ply_cloud;
  tinyply::PlyFile ply_file(in_stream);
  VLOG(3) << "Reading all points.";
  const int xyz_point_count = ply_file.request_properties_from_element(
      "vertex", {"x", "y", "z"}, ply_cloud.getXYZPoints());
  CHECK_GT(xyz_point_count, 0);

  const int intensity_point_count = ply_file.request_properties_from_element(
      "vertex", {FLAGS_phaser_ply_intensity_str}, ply_cloud.getIntentsities());
  CHECK_GT(intensity_point_count, 0);
  VLOG(3) << "Found: " << intensity_point_count;

  const int refl_point_count = ply_file.request_properties_from_element(
      "vertex", {FLAGS_phaser_ply_reflectivity_str},
      ply_cloud.getReflectivities());
  LOG_IF(WARNING, refl_point_count <= 0) << "No reflectivity channel found.";

  const int ambient_point_count = ply_file.request_properties_from_element(
      "vertex", {FLAGS_phaser_ply_ambient_str}, ply_cloud.getAmbientPoints());
  LOG_IF(WARNING, ambient_point_count <= 0) << "No ambient channel found.";

  const int range_point_count = ply_file.request_properties_from_element(
      "vertex", {FLAGS_phaser_ply_range_str}, ply_cloud.getRange());
  LOG_IF(WARNING, range_point_count <= 0) << "No range channel found.";

  VLOG(3) << "Found: " << xyz_point_count << " xyz points, "
          << intensity_point_count << " intensity points, " << refl_point_count
          << " reflectivity points, " << ambient_point_count
          << " ambient points, 0"  // << range_point_count
          << " range points for reading.";

  ply_file.read(in_stream);
  in_stream.close();
  return ply_cloud;
}

}  // namespace data
