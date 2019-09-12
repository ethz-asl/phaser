#include "packlo/common/statistics-manager.h"

namespace common {

StatisticsManager::StatisticsManager(std::string &&name)
	: reference_name_(name) {}

StatisticsManager::StatisticsManager(const std::string &name)
	: reference_name_(name) {}

void StatisticsManager::emplaceValue(std::string&& key, double value) {
	statistics_[key].emplace_back(value);
}

void StatisticsManager::emplaceValue(const std::string& key, double value) {
	statistics_[key].emplace_back(value);
}

std::vector<double> StatisticsManager::getValuesForKey(
		const std::string &key) {
	return statistics_[key];
}

} // namespace common
