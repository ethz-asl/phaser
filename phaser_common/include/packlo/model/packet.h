#pragma once

#include <packlo/model/point.h>

#include <memory>
#include <vector>

namespace model {

class Packet {
  public:
    Packet();

  private:
    std::vector<PointXYZIf> points_;

};

} // namespace model
