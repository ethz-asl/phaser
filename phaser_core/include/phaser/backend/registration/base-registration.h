#pragma once

#include <memory>

#include "phaser/common/statistics-manager.h"
#include "phaser/model/point-cloud.h"
#include "phaser/model/registration-result.h"

namespace phaser_core {

class BaseRegistration {
 public:
  virtual ~BaseRegistration() = default;
  virtual model::RegistrationResult registerPointCloud(
      model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur) = 0;

  virtual void getStatistics(common::StatisticsManager* manager) const noexcept;

 protected:
  BaseRegistration() : statistics_manager_("") {}
  BaseRegistration(const std::string& reference_name)
      : statistics_manager_(reference_name) {}

  common::StatisticsManager statistics_manager_;
};

using BaseRegistrationPtr = std::unique_ptr<BaseRegistration>;

}  // namespace phaser_core
