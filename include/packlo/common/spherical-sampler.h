#pragma once

#include <packlo/model/point-cloud.h>

namespace common {

class SphericalSampler {
  public:
    static std::vector<float> sampleUniformly(model::PointCloud &cloud, 
        const std::size_t bw);

  private:
    static std::vector<model::Point_t> create2BwGrid(
        const std::size_t bw, const bool cartesian = true);
    static std::vector<model::Point_t> convertCartesian(
        std::vector<model::Point_t>& grid);
};
  

}
