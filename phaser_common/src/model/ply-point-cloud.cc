#include "phaser/model/ply-point-cloud.h"

namespace model {

std::vector<float>& PlyPointCloud::getXYZPoints() {
  return xyz_points_;
}

const std::vector<float>& PlyPointCloud::getXYZPoints() const {
  return xyz_points_;
}

std::vector<float>& PlyPointCloud::getIntentsities() {
  return intensities_;
}

const std::vector<float>& PlyPointCloud::getIntentsities() const {
  return intensities_;
}

std::vector<float>& PlyPointCloud::getReflectivities() {
  return reflectivities_;
}

const std::vector<float>& PlyPointCloud::getReflectivities() const {
  return reflectivities_;
}

std::vector<float>& PlyPointCloud::getAmbientPoints() {
  return ambient_points_;
}

const std::vector<float>& PlyPointCloud::getAmbientPoints() const {
  return ambient_points_;
}

std::vector<float>& PlyPointCloud::getRange() {
  return range_;
}

const std::vector<float>& PlyPointCloud::getRange() const {
  return range_;
}

}  // namespace model
