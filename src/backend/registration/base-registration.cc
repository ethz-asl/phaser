#include "packlo/backend/registration/base-registration.h"

namespace registration {

void BaseRegistration::getStatistics(
		common::StatisticsManager* manager) const noexcept {
	manager->mergeManager(statistics_manager_);
}

} // namespace registration
