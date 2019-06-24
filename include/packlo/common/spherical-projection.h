#pragma once 

#include <packlo/model/point-cloud.h>

namespace common {

class SphericalProjection {

public:
  static void convertPointCloud(model::PointCloud &cloud);

private:
  static void naiveProjection(model::PointCloud &cloud);

};

}


