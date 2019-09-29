#pragma once

#include "packlo/common/point-types.h"
#include "packlo/model/function-value.h"

#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/kdtree/kdtree_flann.h>

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

   void getNearestPoints(
			const std::vector<Point_t> &query_points, 
			std::vector<FunctionValue>* function_values) const;

  void transformPointCloud(const Eigen::Matrix4f &T);
  void transformPointCloudCopy(const Eigen::Matrix4f& T, PointCloud& copy);

  PointCloud_tPtr getRawCloud() const;

  Point_t& pointAt(const std::size_t idx);
  const Point_t& pointAt(const std::size_t idx) const;

  std::size_t size() const;
  PointCloud clone() const;

	void initialize_kd_tree();

private:
  PointCloud_tPtr cloud_; 
  pcl::KdTreeFLANN<Point_t> kd_tree_; 

	bool kd_tree_is_initialized_;
  const std::size_t kNeighbors = 1;
};

using PointCloudPtr = std::shared_ptr<PointCloud>;

}
