#pragma once

#include <map>
#include <vector>

namespace common {

class	StatisticsManager {
	public: 
		explicit StatisticsManager(std::string &&name);
		explicit StatisticsManager(const std::string &name);

		void emplaceValue(std::string&& key, double value);
		void emplaceValue(const std::string& key, double value);
		std::vector<double> getValuesForKey(const std::string &key);

	private:
		const std::string reference_name_;
		std::map<std::string, std::vector<double>> statistics_;
};

} // namespace
