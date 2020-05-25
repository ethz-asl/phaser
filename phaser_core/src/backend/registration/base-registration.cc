#include "phaser/backend/registration/base-registration.h"

namespace phaser_core {

void BaseRegistration::getStatistics(
    common::StatisticsManager* manager) const noexcept {
  manager->mergeManager(statistics_manager_);
}

}  // namespace phaser_core
