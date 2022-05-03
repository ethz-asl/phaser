#pragma once

#include <map>
#include <vector>

namespace common {

class StatisticsManager {
 public:
  explicit StatisticsManager(std::string&& name);
  explicit StatisticsManager(const std::string& name);

  void emplaceValue(std::string&& key, double value);
  void emplaceValue(const std::string& key, double value);
  std::vector<double> getValuesForKey(const std::string& key) const;
  std::vector<double> getValuesForKey(std::string&& key) const;
  std::size_t count(std::string&& key) const;

  void mergeManager(const StatisticsManager& manager);

 private:
  const std::string reference_name_;
  std::map<std::string, std::vector<double>> statistics_;
};

}  // namespace common
