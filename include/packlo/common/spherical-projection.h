#pragma once 

#include <packlo/model/point-cloud.h>

namespace common {

class SphericalProjection {

public:
  static void convertPointCloud(model::PointCloud &cloud);
  static model::PointCloud convertPointCloudCopy
    (const model::PointCloud &cloud);

private:
  static void naiveProjection(const model::PointCloud &cloud_in,
      model::PointCloud &cloud_out);

};

}


