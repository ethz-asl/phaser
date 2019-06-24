#include <packlo/model/point-cloud.h>

namespace model {

PointCloud::PointCloud(PointCloud_tPtr cloud) 
  : cloud_(cloud) {
  
}

PointCloud_t::iterator PointCloud::begin() {
  return cloud_->begin();
}

PointCloud_t::iterator PointCloud::end() {
  return cloud_->end();
}


}
