#ifndef INCLUDE_PACKLO_MODEL_POINT_CLOUD_H_
#define INCLUDE_PACKLO_MODEL_POINT_CLOUD_H_

#include "packlo/common/point-types.h"
#include "packlo/model/function-value.h"
#include "packlo/model/point.h"

#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <memory>
#include <string>
#include <vector>

namespace model {

class PointCloud {
 public:
  PointCloud();
  explicit PointCloud(common::PointCloud_tPtr cloud);
  explicit PointCloud(common::ExtractedPointCloud_tPtr cloud);
  explicit PointCloud(const std::string& ply);

  common::PointCloud_t::iterator begin();
  common::PointCloud_t::iterator end();

  void getNearestPoints(
      const std::vector<common::Point_t>& query_points,
      std::vector<FunctionValue>* function_values) const;

  void transformPointCloud(const Eigen::Matrix4f &T);
  void transformPointCloudCopy(
      const Eigen::Matrix4f& T, PointCloud* copy) const;

  common::PointCloud_tPtr getRawCloud() const;
  common::ExtractedPointCloud_tPtr getRawInfoCloud() const;

  common::Point_t& pointAt(const std::size_t idx);
  const common::Point_t& pointAt(const std::size_t idx) const;

  common::ExtractedPoint_t& pointInfoAt(const std::size_t idx);
  const common::ExtractedPoint_t& pointInfoAt(const std::size_t idx) const;

  std::size_t size() const;
  PointCloud clone() const;

  void initialize_kd_tree();
  void writeToFile(std::string&& directory = "");

  void updateInfo(const pcl::IndicesConstPtr indices);
  void updateCloud();

 private:
  void convertInputPointCloud(common::ExtractedPointCloud_tPtr cloud);
  void readFromFile(const std::string& ply);
  common::PointCloud_tPtr cloud_;
  common::ExtractedPointCloud_tPtr cloud_info_;
  pcl::KdTreeFLANN<common::Point_t> kd_tree_;

  bool kd_tree_is_initialized_;
  std::string ply_directory_;
};

using PointCloudPtr = std::shared_ptr<PointCloud>;

}  // namespace model

#endif  // INCLUDE_PACKLO_MODEL_POINT_CLOUD_H_
