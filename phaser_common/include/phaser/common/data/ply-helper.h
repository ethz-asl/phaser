#ifndef PHASER_COMMON_DATA_PLY_HELPER_H_
#define PHASER_COMMON_DATA_PLY_HELPER_H_

#include <string>

#include "phaser/model/ply-point-cloud.h"

namespace data {

class PlyHelper {
 public:
  model::PlyPointCloud readPlyFromFile(const std::string& filename);
};

}  // namespace data

#endif  //  PHASER_COMMON_DATA_PLY_HELPER_H_
