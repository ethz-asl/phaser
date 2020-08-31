#ifndef PHASER_COMMON_SPHERICAL_FEATURE_H_
#define PHASER_COMMON_SPHERICAL_FEATURE_H_

#include <vector>

namespace common {

class SphericalFeature {
 public:
  std::vector<double>& getRangeFeatures();
  std::vector<double>& getIntensityFeatures();
  std::vector<double>& getVisualFeatures();

  const std::vector<double>& getRangeFeatures() const noexcept;
  const std::vector<double>& getIntensityFeatures() const noexcept;
  const std::vector<double>& getVisualFeatures() const noexcept;

 private:
  std::vector<double> range_features_;
  std::vector<double> intensity_features_;
  std::vector<double> visual_features_;
};

}  // namespace common

#endif  // PHASER_COMMON_SPHERICAL_FEATURE_H_
