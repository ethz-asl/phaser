#ifndef PHASER_COMMON_DUAL_QUATERNION_H_
#define PHASER_COMMON_DUAL_QUATERNION_H_

#include <Eigen/Dense>

namespace common {

class DualQuaternion {
 public:
  DualQuaternion() = default;
  explicit DualQuaternion(const Eigen::Quaterniond& q, const Eigen::Vector3d t);
  explicit DualQuaternion(const Eigen::VectorXd& vec);

  Eigen::VectorXd asVec() const;

  Eigen::Quaterniond& getRotation();
  const Eigen::Quaterniond& getRotation() const;
  Eigen::Vector3d& getTranslation();
  const Eigen::Vector3d& getTranslation() const;

 private:
  Eigen::Quaterniond rotation_;
  Eigen::Vector3d translation_;
};

}  // namespace common

#endif  // PHASER_COMMON_DUAL_QUATERNION_H_
