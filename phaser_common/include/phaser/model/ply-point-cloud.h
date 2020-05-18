#ifndef PHASER_MODEL_PLY_POINT_CLOUD_H_
#define PHASER_MODEL_PLY_POINT_CLOUD_H_

#include <vector>

namespace model {

class PlyPointCloud {
 public:
  std::vector<float>& getXYZPoints();
  const std::vector<float>& getXYZPoints() const;

  std::vector<float>& getIntentsities();
  const std::vector<float>& getIntentsities() const;

  std::vector<float>& getReflectivities();
  const std::vector<float>& getReflectivities() const;

  std::vector<float>& getAmbientPoints();
  const std::vector<float>& getAmbientPoints() const;

  std::vector<float>& getRange();
  const std::vector<float>& getRange() const;

 private:
  std::vector<float> xyz_points_;
  std::vector<float> intensities_;
  std::vector<float> reflectivities_;
  std::vector<float> ambient_points_;
  std::vector<float> range_;
};

}  // namespace model

#endif  // PHASER_MODEL_PLY_POINT_CLOUD_H_
