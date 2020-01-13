#ifndef PACKLO_CONTROLLER_DISTRIBUTOR_H_
#define PACKLO_CONTROLLER_DISTRIBUTOR_H_

#include "packlo/backend/registration/base-registration.h"
#include "packlo/common/data/datasource-ros.h"
#include "packlo/common/statistics-manager.h"
#include "packlo/experiments/experiment-handler.h"
#include "packlo/model/point-cloud.h"

#include <memory>
#include <string>

namespace controller {

class Distributor {
 public:
  explicit Distributor(const data::DatasourcePtr& ds);

  void updateStatistics();
  void getStatistics(common::StatisticsManager*) const noexcept;

  void setRegistrationAlgorithm(std::string&& algorithm);
  void setRegistrationAlgorithm(const std::string& algorithm);
  void initializeRegistrationAlgorithm();
  void shutdown();

 private:
  void subscribeToTopics();
  void pointCloudCallback(const model::PointCloudPtr& cloud);
  void registerPointCloud(const model::PointCloudPtr& cloud);
  void preprocessPointCloud(const model::PointCloudPtr& cloud);

  data::DatasourcePtr ds_;
  registration::BaseRegistrationPtr registrator_;
  model::PointCloudPtr prev_point_cloud_;
  std::string registration_algorithm_;

  experiments::ExperimentHandlerPtr experiment_handler_;

  // Statistics
  const std::string kManagerReferenceName = "Distributor";
  common::StatisticsManager statistics_manager_;
};

}  // namespace controller

#endif  // PACKLO_CONTROLLER_DISTRIBUTOR_H_
