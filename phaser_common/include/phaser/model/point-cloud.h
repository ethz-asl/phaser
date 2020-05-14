#ifndef PHASER_MODEL_POINT_CLOUD_H_
#define PHASER_MODEL_POINT_CLOUD_H_

#include "phaser/common/point-types.h"
#include "phaser/model/function-value.h"
#include "phaser/model/point.h"

#include <pcl/common/projection_matrix.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include <memory>
#include <string>
#include <vector>

namespace model {

class PointCloud {
 public:
  PointCloud();
  explicit PointCloud(common::PointCloud_tPtr cloud);
  explicit PointCloud(const std::string& ply);
  explicit PointCloud(const std::vector<common::Point_t>& points);

  common::PointCloud_t::iterator begin();
  common::PointCloud_t::iterator end();

  void getNearestPoints(
      const std::vector<common::Point_t>& query_points,
      std::vector<FunctionValue>* function_values) const;

  void transformPointCloud(const Eigen::Matrix4f& T);
  void transformPointCloudCopy(
      const Eigen::Matrix4f& T, PointCloud* copy) const;

  common::PointCloud_tPtr getRawCloud() const;
  common::PointCloud_tPtr& getRawCloud();
  common::PointCloud_tPtr getRawInfoCloud() const;
  common::PointCloud_tPtr& getRawInfoCloud();
  bool hasInfoCloud() const;

  common::Point_t& pointAt(const std::size_t idx);
  const common::Point_t& pointAt(const std::size_t idx) const;
  common::Point_t& infoPointAt(const std::size_t idx);
  const common::Point_t& infoPointAt(const std::size_t idx) const;

  std::size_t size() const;
  PointCloud clone() const;

  void setRange(const double range, const uint32_t i);
  double getRange(const uint32_t i) const;

  void initialize_kd_tree();
  void writeToFile(std::string&& directory = "");

  std::string getPlyReadDirectory() const noexcept;

 private:
  void readFromFile(const std::string& ply);
  void sampleNearestWithoutCloudInfo(
      const uint32_t idx, const std::vector<int>& pointIdxNKNSearch,
      const std::vector<float>& pointNKNSquaredDistance,
      std::vector<FunctionValue>* function_values) const;
  void sampleNearestWithCloudInfo(
      const uint32_t idx, const std::vector<int>& pointIdxNKNSearch,
      const std::vector<float>& pointNKNSquaredDistance,
      std::vector<FunctionValue>* function_values) const;

  common::PointCloud_tPtr cloud_;
  common::PointCloud_tPtr info_cloud_;
  pcl::KdTreeFLANN<common::Point_t> kd_tree_;

  bool kd_tree_is_initialized_;
  std::string ply_directory_;
  std::vector<double> ranges_;
  std::string ply_read_directory_;
};

using PointCloudPtr = std::shared_ptr<PointCloud>;

}  // namespace model

#endif  // PHASER_MODEL_POINT_CLOUD_H_
