#include "phaser/common/statistics-manager.h"

#include <glog/logging.h>

namespace common {

StatisticsManager::StatisticsManager(std::string&& name)
    : reference_name_(name) {}

StatisticsManager::StatisticsManager(const std::string& name)
    : reference_name_(name) {}

void StatisticsManager::emplaceValue(std::string&& key, double value) {
  statistics_[key].emplace_back(value);
}

void StatisticsManager::emplaceValue(const std::string& key, double value) {
  statistics_[key].emplace_back(value);
}

std::vector<double> StatisticsManager::getValuesForKey(
    const std::string& key) const {
  if (statistics_.count(key) == 0)
    return std::vector<double>();
  return statistics_.at(key);
}

std::vector<double> StatisticsManager::getValuesForKey(
    std::string&& key) const {
  if (statistics_.count(key) == 0)
    return std::vector<double>();
  return statistics_.at(key);
}

void StatisticsManager::mergeManager(const StatisticsManager& manager) {
  statistics_.insert(manager.statistics_.begin(), manager.statistics_.end());
}

std::size_t StatisticsManager::count(std::string&& key) const {
  return statistics_.count(key);
}

}  // namespace common
