#include "packlo/experiments/experiment-handler.h"

namespace experiments {

ExperimentHandler::ExperimentHandler(
    registration::BaseRegistrationPtr&& registrator)
    : registrator_(std::move(registrator)), prev_point_cloud_(nullptr) {}

void ExperimentHandler::shutdown() {
  if (states_.empty())
    return;
  writeResultsToFile();
}

void ExperimentHandler::runExperiment1(const model::PointCloudPtr& cloud) {
  if (prev_point_cloud_ == nullptr) {
    prev_point_cloud_ = cloud;
    return;
  }

  // Register the point clouds.
  model::RegistrationResult result =
      registrator_->registerPointCloud(prev_point_cloud_, cloud);
  appendResult(result);

  // Wait for the next pair.
  prev_point_cloud_ = nullptr;
}

void ExperimentHandler::appendResult(const model::RegistrationResult& result) {
  const Eigen::VectorXd dq = result.getStateAsVec();
  states_.emplace_back(std::move(dq));
}

void ExperimentHandler::writeResultsToFile() {}

}  // namespace experiments
