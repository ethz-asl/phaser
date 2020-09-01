#include "phaser/common/spherical-feature.h"

namespace common {

std::vector<double>& SphericalFeature::getRangeFeatures() {
  return range_features_;
}

std::vector<double>& SphericalFeature::getIntensityFeatures() {
  return intensity_features_;
}

std::vector<double>& SphericalFeature::getVisualFeatures() {
  return visual_features_;
}

std::vector<double>& SphericalFeature::getRangeTransformed() {
  return range_transformed_;
}

std::vector<double>& SphericalFeature::getIntensityTransformed() {
  return intensity_transformed_;
}

std::vector<double>& SphericalFeature::getVisualTransformed() {
  return visual_transformed_;
}

const std::vector<double>& SphericalFeature::getRangeFeatures() const noexcept {
  return range_features_;
}

const std::vector<double>& SphericalFeature::getIntensityFeatures() const
    noexcept {
  return intensity_features_;
}

const std::vector<double>& SphericalFeature::getVisualFeatures() const
    noexcept {
  return visual_features_;
}

const std::vector<double>& SphericalFeature::getRangeTransformed() const
    noexcept {
  return range_transformed_;
}

const std::vector<double>& SphericalFeature::getIntensityTransformed() const
    noexcept {
  return intensity_transformed_;
}

const std::vector<double>& SphericalFeature::getVisualTransformed() const
    noexcept {
  return visual_transformed_;
}

}  // namespace common
