#ifndef PHASER_EXPERIMENTS_EXPERIMENT_HANDLER_H_
#define PHASER_EXPERIMENTS_EXPERIMENT_HANDLER_H_

#include "phaser/backend/registration/sph-opt-registration.h"
#include "phaser/model/point-cloud.h"

#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace phaser_core {

class ExperimentHandler {
 public:
  ExperimentHandler();
  void shutdown();

  void runExperiment1(const model::PointCloudPtr& cloud);
  void runExperiment3(const model::PointCloudPtr& cloud);
  void runExperiment4(const model::PointCloudPtr& cloud);

 private:
  void readTruth();
  void appendResult(const model::RegistrationResult& result);
  void writeResultsToFile();
  void translateToSensorFrame(const model::PointCloudPtr& cloud);
  void translateToOdomFrame(const model::PointCloudPtr& cloud);

  void rotateToSensorFrame(const model::PointCloudPtr& cloud);
  void rotateToOdomFrame(const model::PointCloudPtr& cloud);

  SphOptRegistrationPtr registrator_;
  model::PointCloudPtr prev_point_cloud_;
  std::vector<Eigen::VectorXd> states_;
  Eigen::MatrixXd gt_;
  uint16_t n_registered_ = 0u;
};

using ExperimentHandlerPtr = std::unique_ptr<ExperimentHandler>;

}  // namespace phaser_core

#endif  // PHASER_EXPERIMENTS_EXPERIMENT_HANDLER_H_
