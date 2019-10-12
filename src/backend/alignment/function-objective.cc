#include "packlo/backend/alignment/function-objective.h"

#include <Eigen/Dense>
#include <glog/logging.h>

namespace alignment {

double FunctionObjective::optimize(const std::vector<double>& x) {
  CHECK(x.size() == 3);
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0,3) = x[0];
  T(1,3) = x[1];
  T(2,3) = x[2];
  VLOG(1) << "in objective with: " 
    << x[0] << ", " << x[1] << ", " << x[2];

  model::PointCloud transformed_cloud;
  convertFunctionValuesToPointcloud(*f_cur_, transformed_cloud);
  transformed_cloud.transformPointCloud(T);
  //cloud_cur_->transformPointCloudCopy(T, transformed_cloud);

  Eigen::MatrixXf transformed_points = 
    transformed_cloud.getRawCloud()->getMatrixXfMap();

  Eigen::MatrixXf prev_points; 
  convertFunctionValuesToPointMatrix(*f_prev_, prev_points);
    //cloud_prev_->getRawCloud()->getMatrixXfMap();
  transformed_points.conservativeResize(3, transformed_points.cols());
  prev_points.conservativeResize(3, prev_points.cols());

  /*
  VLOG(1) << "function values: " << f_prev_->size() << " vs " << f_cur_->size();
  VLOG(1) << "transformed: " << transformed_points.rows() << " rows and "
    << transformed_points.cols() << " cols.\n" << transformed_points;
  VLOG(1) << "----------------------------------------------------------------";
  VLOG(1) << "prev_points: " << prev_points.rows() << " rows and "
    << prev_points.cols() << " cols. \n" << prev_points;;
  VLOG(1) << "----------------------------------------------------------------";
  */

  CHECK(transformed_points.rows() == prev_points.rows());
  CHECK(transformed_points.cols() == prev_points.cols());
  float error = (prev_points.array() - transformed_points.array()).pow(2).sum();
  VLOG(1) << "Current error: " << error;
  return (prev_points.array() - transformed_points.array()).pow(2).sum();
}

void FunctionObjective::calculateGrad(std::vector<double>&) {

}

void FunctionObjective::convertFunctionValuesToPointMatrix(
    const std::vector<model::FunctionValue>& values, 
    Eigen::MatrixXf& matrix) {
  const std::size_t n_values = values.size();
  matrix.resize(3, n_values);
  for (std::size_t i = 0u; i < n_values; ++i) {
    common::Point_t avg = values[i].getAveragedPoint();
    matrix(1, i) = avg.x;
    matrix(2, i) = avg.y;
    matrix(3, i) = avg.z;
  }
}

void FunctionObjective::convertFunctionValuesToPointcloud(
    const std::vector<model::FunctionValue>& values, 
    model::PointCloud& cloud) {
  common::PointCloud_tPtr raw_cloud = cloud.getRawCloud();
  raw_cloud->reserve(values.size());
  for (const model::FunctionValue& value : values) {
    (*raw_cloud) += *value.getAllPoints();
    //raw_cloud->push_back(value.getAveragedPoint());
  }
}

} // namespace alignment
