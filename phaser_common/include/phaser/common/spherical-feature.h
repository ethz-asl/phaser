#ifndef PHASER_COMMON_SPHERICAL_FEATURE_H_
#define PHASER_COMMON_SPHERICAL_FEATURE_H_

#include <vector>

namespace common {

class SphericalFeature {
 public:
  std::vector<double>& getRangeFeatures();
  std::vector<double>& getIntensityFeatures();
  std::vector<double>& getVisualFeatures();

  std::vector<double>& getRangeTransformed();
  std::vector<double>& getIntensityTransformed();
  std::vector<double>& getVisualTransformed();

  const std::vector<double>& getRangeFeatures() const noexcept;
  const std::vector<double>& getIntensityFeatures() const noexcept;
  const std::vector<double>& getVisualFeatures() const noexcept;

  const std::vector<double>& getRangeTransformed() const noexcept;
  const std::vector<double>& getIntensityTransformed() const noexcept;
  const std::vector<double>& getVisualTransformed() const noexcept;

 private:
  std::vector<double> range_features_;
  std::vector<double> intensity_features_;
  std::vector<double> visual_features_;

  std::vector<double> range_transformed_;
  std::vector<double> intensity_transformed_;
  std::vector<double> visual_transformed_;
};

}  // namespace common

#endif  // PHASER_COMMON_SPHERICAL_FEATURE_H_
