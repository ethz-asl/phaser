#include "phaser/common/dual-quaternion.h"

#include <glog/logging.h>

namespace common {

DualQuaternion::DualQuaternion(
    const Eigen::Quaterniond& q, const Eigen::Vector3d t)
    : rotation_(q), translation_(t) {}

DualQuaternion::DualQuaternion(const Eigen::VectorXd& vec) {
  CHECK_EQ(vec.rows(), 8);
  rotation_ = Eigen::Quaterniond(vec(0), vec(1), vec(2), vec(3));
  translation_ = Eigen::Vector3d(vec(5), vec(6), vec(7));
}

Eigen::VectorXd DualQuaternion::asVec() const {
  Eigen::VectorXd vec(8);
  vec << rotation_.w(), rotation_.x(), rotation_.y(), rotation_.z(), 0,
      translation_(0), translation_(1), translation_(2);
  return vec;
}

Eigen::Quaterniond& DualQuaternion::getRotation() {
  return rotation_;
}

const Eigen::Quaterniond& DualQuaternion::getRotation() const {
  return rotation_;
}

Eigen::Vector3d& DualQuaternion::getTranslation() {
  return translation_;
}

const Eigen::Vector3d& DualQuaternion::getTranslation() const {
  return translation_;
}

}  // namespace common
