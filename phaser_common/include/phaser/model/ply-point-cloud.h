#ifndef PHASER_MODEL_PLY_POINT_CLOUD_H_
#define PHASER_MODEL_PLY_POINT_CLOUD_H_

#include <vector>

namespace model {

class PlyPointCloud {
 public:
  std::vector<double>& getXYZPoints();
  const std::vector<double>& getXYZPoints() const;

  std::vector<double>& getIntentsities();
  const std::vector<double>& getIntentsities() const;

  std::vector<double>& getReflectivities();
  const std::vector<double>& getReflectivities() const;

  std::vector<double>& getAmbientPoints();
  const std::vector<double>& getAmbientPoints() const;

  std::vector<double>& getRange();
  const std::vector<double>& getRange() const;

 private:
  std::vector<double> xyz_points_;
  std::vector<double> intensities_;
  std::vector<double> reflectivities_;
  std::vector<double> ambient_points_;
  std::vector<double> range_;
};

}  // namespace model

#endif  // PHASER_MODEL_PLY_POINT_CLOUD_H_
