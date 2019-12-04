#include "packlo/backend/alignment/cloud-objective.h"

#include <Eigen/Dense>
#include <glog/logging.h>

namespace alignment {

double CloudObjective::optimize(const std::vector<double>& x) {
  CHECK_EQ(x.size(), 3);
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0, 3) = x[0];
  T(1, 3) = x[1];
  T(2, 3) = x[2];

  model::PointCloud transformed_cloud;
  cloud_cur_->transformPointCloudCopy(T, &transformed_cloud);
  Eigen::MatrixXf transformed_points =
      transformed_cloud.getRawCloud()->getMatrixXfMap();

  Eigen::MatrixXf prev_points =
    cloud_prev_->getRawCloud()->getMatrixXfMap();
  std::size_t min_size =
      std::min(transformed_points.cols(), prev_points.cols());
  transformed_points.conservativeResize(3, min_size);
  prev_points.conservativeResize(3, min_size);

  CHECK(transformed_points.rows() == prev_points.rows());
  CHECK(transformed_points.cols() == prev_points.cols());
  float error = (prev_points.array() - transformed_points.array()).pow(2).sum();
  return (prev_points.array() - transformed_points.array()).pow(2).sum();
}

void CloudObjective::calculateGrad(std::vector<double>&) {}

}  // namespace alignment
