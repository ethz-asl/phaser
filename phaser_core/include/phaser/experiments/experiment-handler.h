#ifndef PACKLO_EXPERIMENTS_EXPERIMENT_HANDLER_H_
#define PACKLO_EXPERIMENTS_EXPERIMENT_HANDLER_H_

#include "phaser/backend/registration/base-registration.h"
#include "phaser/backend/registration/sph-registration.h"
#include "phaser/model/point-cloud.h"
#include "phaser/backend/registration/g-icp-registration.h"


#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace experiments {

class ExperimentHandler {
 public:
  // explicit ExperimentHandler(registration::BaseRegistrationPtr&&
  // registrator);
  ExperimentHandler();
  void shutdown();

  void runExperiment1(const model::PointCloudPtr& cloud);
  void runExperiment3(const model::PointCloudPtr& cloud);
  void runExperiment4(const model::PointCloudPtr& cloud);
  void runExperimentGICP(const model::PointCloudPtr& cloud);

 private:
  void readTruth();
  void appendResult(const model::RegistrationResult& result);
  void writeResultsToFile();
  void translateToSensorFrame(const model::PointCloudPtr& cloud);
  void translateToOdomFrame(const model::PointCloudPtr& cloud);

  void rotateToSensorFrame(const model::PointCloudPtr& cloud);
  void rotateToOdomFrame(const model::PointCloudPtr& cloud);

  registration::SphRegistrationPtr registrator_;
  model::PointCloudPtr prev_point_cloud_;
  std::vector<Eigen::VectorXd> states_;
  Eigen::MatrixXd gt_;
  uint16_t n_registered_ = 0u;
  registration::GIcpRegistration gicp_reg_;
  std::vector<Eigen::Matrix4f> gicp_states_;
};

using ExperimentHandlerPtr = std::unique_ptr<ExperimentHandler>;

}  // namespace experiments

#endif  // PACKLO_EXPERIMENTS_EXPERIMENT_HANDLER_H_
