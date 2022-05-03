#include "phaser/common/data/file-system-helper.h"

#include <boost/filesystem.hpp>
#include <glog/logging.h>

namespace data {

void FileSystemHelper::readDirectory(
    const std::string& directory, std::vector<std::string>* files) {
  boost::filesystem::path p(directory);
  if (!boost::filesystem::exists(p)) {
    LOG(FATAL) << "PLY directory does not exist!";
  }

  boost::filesystem::directory_iterator start(p);
  boost::filesystem::directory_iterator end;
  std::transform(
      start, end, std::back_inserter(*files),
      [](const boost::filesystem::directory_entry& entry) {
        return entry.path().leaf().string();
      });
  std::sort(files->begin(), files->end());
}

}  // namespace data
