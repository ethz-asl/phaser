#pragma once

#include <memory>
#include <vector>

#include "phaser/model/point.h"

namespace model {

class Packet {
 public:
  Packet();

 private:
  std::vector<PointXYZIf> points_;
};

}  // namespace model
