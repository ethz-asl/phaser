#pragma once

#include "packlo/common/point-types.h"
#include "packlo/model/function-value.h"

#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <memory>
#include <vector>

namespace model {

class PointCloud {

public:
  explicit PointCloud(common::PointCloud_tPtr cloud);

  common::PointCloud_t::iterator begin(); 
  common::PointCloud_t::iterator end();

   void getNearestPoints(
			const std::vector<common::Point_t> &query_points, 
			std::vector<FunctionValue>* function_values) const;

  void transformPointCloud(const Eigen::Matrix4f &T);
  void transformPointCloudCopy(const Eigen::Matrix4f& T, PointCloud& copy);

  common::PointCloud_tPtr getRawCloud() const;

  common::Point_t& pointAt(const std::size_t idx);
  const common::Point_t& pointAt(const std::size_t idx) const;

  std::size_t size() const;
  PointCloud clone() const;

	void initialize_kd_tree();
	void writeToFile();

private:
	common::PointCloud_tPtr cloud_; 
  pcl::KdTreeFLANN<common::Point_t> kd_tree_; 

	bool kd_tree_is_initialized_;
  const std::size_t kNeighbors = 1;
};

using PointCloudPtr = std::shared_ptr<PointCloud>;

}
