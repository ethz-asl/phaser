#ifndef PACKLO_EXPERIMENTS_EXPERIMENT_HANDLER_H_
#define PACKLO_EXPERIMENTS_EXPERIMENT_HANDLER_H_

#include "packlo/backend/registration/base-registration.h"
#include "packlo/model/point-cloud.h"

#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace experiments {

class ExperimentHandler {
 public:
  explicit ExperimentHandler(registration::BaseRegistrationPtr&& registrator);
  void shutdown();

  void runExperiment1(const model::PointCloudPtr& cloud);

 private:
  void appendResult(const model::RegistrationResult& result);
  void writeResultsToFile();

  registration::BaseRegistrationPtr registrator_;
  model::PointCloudPtr prev_point_cloud_;
  std::vector<Eigen::VectorXd> states_;
};

using ExperimentHandlerPtr = std::unique_ptr<ExperimentHandler>;

}  // namespace experiments

#endif  // PACKLO_EXPERIMENTS_EXPERIMENT_HANDLER_H_
