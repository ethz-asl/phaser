#include "packlo/backend/alignment/objective.h"

#include <Eigen/Dense>
#include <glog/logging.h>

namespace alignment {

double Objective::optimize(const std::vector<double>& x, 
    std::vector<double>&) {
  CHECK(x.size() == 3);
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0,3) = x[0];
  T(1,3) = x[1];
  T(2,3) = x[2];
  VLOG(1) << "in objective with: " 
    << x[0] << ", " << x[1] << ", " << x[2];

  model::PointCloud transformed_cloud;
  cloud_cur_->transformPointCloudCopy(T, transformed_cloud);
  Eigen::MatrixXf transformed_points = 
    transformed_cloud.getRawCloud()->getMatrixXfMap();

  Eigen::MatrixXf prev_points = 
    cloud_prev_->getRawCloud()->getMatrixXfMap();

  CHECK(transformed_points.rows() == prev_points.rows());
  CHECK(transformed_points.cols() == prev_points.cols());
  float error = (prev_points.array() - transformed_points.array()).pow(2).sum();
  VLOG(1) << "Current error: " << error;
  return (prev_points.array() - transformed_points.array()).pow(2).sum();
}

void Objective::calculateGrad(std::vector<double>&) {

}

void Objective::setPrevious(const model::PointCloud& cloud_prev, 
    const std::vector<model::FunctionValue>& f_prev) {
  cloud_prev_ = &cloud_prev;
  f_prev_ = &f_prev;

}

void Objective::setCurrent(const model::PointCloud& cloud_cur, 
    const std::vector<model::FunctionValue>& f_cur) {
  cloud_cur_ = &cloud_cur;
  f_cur_ = &f_cur;
}

} // namespace alignment
