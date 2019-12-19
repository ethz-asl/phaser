#ifndef PACKLO_COMMON_DATA_FILE_SYSTEM_HELPER_H_
#define PACKLO_COMMON_DATA_FILE_SYSTEM_HELPER_H_

#include <vector>
#include <string>

namespace data {

class FileSystemHelper {
 public:
  static void readDirectory(
      const std::string& directory, std::vector<std::string>* files);
};

}  // namespace data

#endif  // PACKLO_COMMON_DATA_FILE_SYSTEM_HELPER_H_
