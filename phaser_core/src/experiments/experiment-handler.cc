#include "packlo/experiments/experiment-handler.h"
#include "packlo/common/translation-utils.h"
#include "packlo/visualization/debug-visualizer.h"
#include "packlo/visualization/plotty-visualizer.h"

#include <fstream>
#include <glog/logging.h>
#include <memory>
#include <sstream>

DEFINE_string(
    output_file, "experiment_results.txt",
    "Declares the name of the ouput file.");

namespace experiments {

ExperimentHandler::ExperimentHandler() : prev_point_cloud_(nullptr) {
  registrator_ = std::make_unique<registration::SphRegistration>();
  readTruth();
}

void ExperimentHandler::shutdown() {
  if (states_.empty())
    return;
  writeResultsToFile();
}

void ExperimentHandler::runExperiment1(const model::PointCloudPtr& cloud) {
  CHECK_NOTNULL(registrator_);
  if (prev_point_cloud_ == nullptr) {
    prev_point_cloud_ = cloud;
    return;
  }
  // Register the point clouds.
  VLOG(2) << " === Ground truth =============================";
  VLOG(2) << gt_.col(n_registered_).transpose();
  VLOG(1) << "Cloud1: " << prev_point_cloud_->getPlyReadDirectory();
  VLOG(1) << "Cloud2: " << cloud->getPlyReadDirectory();

  translateToSensorFrame(cloud);
  model::RegistrationResult result =
      registrator_->estimateRotation(prev_point_cloud_, cloud);
  translateToOdomFrame(cloud);
  registrator_->estimateTranslation(prev_point_cloud_, &result);

  /*
  visualization::DebugVisualizer::getInstance()
    .visualizePointCloudDiff(*prev_point_cloud_, *result.getRegisteredCloud());
    */
  appendResult(result);

  // Wait for the next pair.
  prev_point_cloud_ = nullptr;
  ++n_registered_;
}

void ExperimentHandler::runExperiment3(const model::PointCloudPtr& cloud) {
  CHECK_NOTNULL(registrator_);
  if (prev_point_cloud_ == nullptr) {
    prev_point_cloud_ = cloud;
    return;
  }
  model::RegistrationResult result =
      registrator_->estimateRotation(prev_point_cloud_, cloud);
  visualization::DebugVisualizer::getInstance().visualizePointCloudDiff(
      *prev_point_cloud_, *result.getRegisteredCloud());
  visualization::PlottyVisualizer::getInstance()
      .createPlotFor(result.getRotationCorrelation())
      .storeToFile(result.getRotationCorrelation());

  appendResult(result);
  prev_point_cloud_ = nullptr;
  ++n_registered_;
}

void ExperimentHandler::runExperiment4(const model::PointCloudPtr& cloud) {
  model::RegistrationResult result =
      registrator_->estimateRotation(prev_point_cloud_, cloud);
  registrator_->estimateTranslation(prev_point_cloud_, &result);
}

void ExperimentHandler::readTruth() {
  const std::string gt =
      "/home/berlukas/Documents/workspace/phaser_ws/src/packlo/"
      "phaser_experimental/phaser_prototype/ply/ground_truth.txt";
  std::ifstream input_gt(gt);
  std::vector<double> result;
  std::string line;

  uint16_t rows = 0;
  while (std::getline(input_gt, line)) {
    std::stringstream lineStream(line);
    std::string cell;
    while (std::getline(lineStream, cell, ',')) {
      result.emplace_back(std::stod(cell));
    }
    ++rows;
  }
  gt_ = Eigen::Map<const Eigen::Matrix<
      double, Eigen::MatrixXd::RowsAtCompileTime,
      Eigen::MatrixXd::ColsAtCompileTime, Eigen::RowMajor>>(
      result.data(), rows, result.size() / rows);
}

void ExperimentHandler::appendResult(const model::RegistrationResult& result) {
  Eigen::VectorXd dq = result.getStateAsVec();
  states_.emplace_back(dq);
}

void ExperimentHandler::writeResultsToFile() {
  VLOG(1) << "Writing experiment results to " << FLAGS_output_file;
  std::ofstream out_file(FLAGS_output_file);
  for (const Eigen::VectorXd& vec : states_) {
    /*
    out_file << vec.transpose();
    out_file << ";\n";
    */
    const uint8_t n_vec = 8;
    for (uint8_t i = 0u; i < n_vec; ++i) {
      if (i == n_vec - 1)
        out_file << vec[i] << "\n";
      else
        out_file << vec[i] << ",";
    }
  }
}

void ExperimentHandler::translateToSensorFrame(
    const model::PointCloudPtr& cloud) {
  Eigen::Vector3d t_vec = gt_.block(3, n_registered_, 3, 1);
  VLOG(1) << "t_vec: " << t_vec;
  common::TranslationUtils::TranslateXYZ(
      cloud, -t_vec(0), -t_vec(1), -t_vec(2));
}

void ExperimentHandler::translateToOdomFrame(
    const model::PointCloudPtr& cloud) {
  Eigen::Vector3d t_vec = gt_.block(3, n_registered_, 3, 1);
  VLOG(1) << "t_vec odom: " << t_vec;
  common::TranslationUtils::TranslateXYZ(cloud, t_vec(0), t_vec(1), t_vec(2));
}

}  // namespace experiments
