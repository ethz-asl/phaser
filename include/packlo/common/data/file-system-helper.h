#pragma once

#include <vector>
#include <string>

namespace data {

class FileSystemHelper {
	public:
		static void readDirectory(const std::string& directory,
			std::vector<std::string>* files);
};

} // namespace data
