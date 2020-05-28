#include "phaser/model/ply-point-cloud.h"

namespace model {

std::vector<double>& PlyPointCloud::getXYZPoints() {
  return xyz_points_;
}

const std::vector<double>& PlyPointCloud::getXYZPoints() const {
  return xyz_points_;
}

std::vector<double>& PlyPointCloud::getIntentsities() {
  return intensities_;
}

const std::vector<double>& PlyPointCloud::getIntentsities() const {
  return intensities_;
}

std::vector<double>& PlyPointCloud::getReflectivities() {
  return reflectivities_;
}

const std::vector<double>& PlyPointCloud::getReflectivities() const {
  return reflectivities_;
}

std::vector<double>& PlyPointCloud::getAmbientPoints() {
  return ambient_points_;
}

const std::vector<double>& PlyPointCloud::getAmbientPoints() const {
  return ambient_points_;
}

std::vector<double>& PlyPointCloud::getRange() {
  return range_;
}

const std::vector<double>& PlyPointCloud::getRange() const {
  return range_;
}

}  // namespace model
