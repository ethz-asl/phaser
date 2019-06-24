#pragma once

#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <memory>

namespace model {

using Point_t = pcl::PointXYZI;
using PointCloud_t = pcl::PointCloud<Point_t>;
using PointCloud_tPtr = std::shared_ptr<pcl::PointCloud<Point_t>>;

class PointCloud {

public:
  explicit PointCloud(PointCloud_tPtr cloud);

  PointCloud_t::iterator begin(); 
  PointCloud_t::iterator end();


private:
  PointCloud_tPtr cloud_; 
};

}
