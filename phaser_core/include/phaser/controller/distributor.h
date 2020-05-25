#ifndef PHASER_CONTROLLER_DISTRIBUTOR_H_
#define PHASER_CONTROLLER_DISTRIBUTOR_H_

#include <memory>
#include <string>
#include <vector>

#include "phaser/backend/registration/base-registration.h"
#include "phaser/common/data/datasource-ros.h"
#include "phaser/common/statistics-manager.h"
#include "phaser/experiments/experiment-handler.h"
#include "phaser/model/point-cloud.h"

namespace phaser_core {

class Distributor {
 public:
  explicit Distributor(
      const data::DatasourcePtr& ds, BaseRegistrationPtr&& registration);

  void updateStatistics();
  void getStatistics(common::StatisticsManager*) const noexcept;

  void setRegistrationAlgorithm(std::string&& algorithm);
  void setRegistrationAlgorithm(const std::string& algorithm);
  void shutdown();

 private:
  void subscribeToTopics();
  void initializeRegistrationAlgorithm();
  void pointCloudCallback(const model::PointCloudPtr& cloud);
  model::RegistrationResult registerPointCloud(
      const model::PointCloudPtr& cloud);
  void preprocessPointCloud(const model::PointCloudPtr& cloud);
  void appendResult(const model::RegistrationResult& result);
  void writeResultsToFile();

  data::DatasourcePtr ds_;
  BaseRegistrationPtr registrator_;
  model::PointCloudPtr prev_point_cloud_;
  std::string registration_algorithm_;

  ExperimentHandlerPtr experiment_handler_;

  // Statistics
  const std::string kManagerReferenceName = "Distributor";
  common::StatisticsManager statistics_manager_;
  uint32_t cloud_counter_;
  std::vector<Eigen::VectorXd> states_;
};

}  // namespace phaser_core

#endif  // PHASER_CONTROLLER_DISTRIBUTOR_H_
