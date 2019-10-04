#pragma once

#include "packlo/model/point-cloud.h"
#include <vector>

namespace common {

class SphericalSampler {
  public:
    SphericalSampler(const int bandwith);
    void sampleUniformly(
        const model::PointCloud &cloud, 
        std::vector<model::FunctionValue>* grid);

    void initialize(const int bandwith);
    int getInitializedBandwith() const noexcept;

  private:
    std::vector<common::Point_t> create2BwGrid(
        const std::size_t bw);
    std::vector<common::Point_t> convertCartesian(
        const std::vector<common::Point_t>& grid);

    std::vector<common::Point_t> cartesian_grid_;
    bool is_initialized_;
    int bandwith_;
};
  

}
