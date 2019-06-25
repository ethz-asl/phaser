#pragma once

#include <packlo/common/point-types.h>

#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <memory>
#include <vector>

namespace model {

//using Point_t = ::OusterPointType;
using Point_t = pcl::PointXYZI;
using PointCloud_t = pcl::PointCloud<Point_t>;
using PointCloud_tPtr = pcl::PointCloud<Point_t>::Ptr;

class PointCloud {

public:
  explicit PointCloud(PointCloud_tPtr cloud);

  PointCloud_t::iterator begin(); 
  PointCloud_t::iterator end();

  std::vector<float> getNearestPoints(const std::vector<Point_t> &query_points);

  void transformPointCloud(const Eigen::Matrix4f &T);
  void transformPointCloudCopy(const Eigen::Matrix4f& T, PointCloud& copy);

  PointCloud_tPtr getRawCloud() const;


private:
  PointCloud_tPtr cloud_; 

  const std::size_t kNeighbors = 1;
};

}
