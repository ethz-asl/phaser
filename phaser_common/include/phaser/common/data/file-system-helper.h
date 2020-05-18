#ifndef PHASER_COMMON_DATA_FILE_SYSTEM_HELPER_H_
#define PHASER_COMMON_DATA_FILE_SYSTEM_HELPER_H_

#include <string>
#include <vector>

namespace data {

class FileSystemHelper {
 public:
  static void readDirectory(
      const std::string& directory, std::vector<std::string>* files);
};

}  // namespace data

#endif  // PHASER_COMMON_DATA_FILE_SYSTEM_HELPER_H_
