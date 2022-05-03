#include "phaser/common/metric-utils.h"

#include <pcl/search/kdtree.h>

namespace common {

float MetricUtils::HausdorffDistance(
    const model::PointCloudPtr& cloud_a, const model::PointCloudPtr& cloud_b) {
  // Compare A to B: sup_a inf_b d(a,b)
  pcl::search::KdTree<common::Point_t> tree_b;
  tree_b.setInputCloud(cloud_b->getRawCloud());
  float max_dist_a = -std::numeric_limits<float>::max();
  for (const common::Point_t& point : cloud_a->getRawCloud()->points) {
    std::vector<int> indices(1);
    std::vector<float> sqr_distances(1);

    tree_b.nearestKSearch(point, 1, indices, sqr_distances);
    if (sqr_distances[0] > max_dist_a)
      max_dist_a = sqr_distances[0];
  }

  // compare B to A: sup_a inf_b d(a,b)
  pcl::search::KdTree<common::Point_t> tree_a;
  tree_a.setInputCloud(cloud_a->getRawCloud());
  float max_dist_b = -std::numeric_limits<float>::max();
  for (const common::Point_t& point : cloud_b->getRawCloud()->points) {
    std::vector<int> indices(1);
    std::vector<float> sqr_distances(1);

    tree_a.nearestKSearch(point, 1, indices, sqr_distances);
    if (sqr_distances[0] > max_dist_b)
      max_dist_b = sqr_distances[0];
  }

  max_dist_a = std::sqrt(max_dist_a);
  max_dist_b = std::sqrt(max_dist_b);

  return std::max(max_dist_a, max_dist_b);
}

}  // namespace common
