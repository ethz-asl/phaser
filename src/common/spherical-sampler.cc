#include <packlo/common/spherical-sampler.h>
#include <cmath>

namespace common {

std::vector<float> SphericalSampler::sampleUniformly(model::PointCloud &cloud,
    const std::size_t bw) {
  std::vector<model::Point_t> sample_angles = create2BwGrid(bw);
  return cloud.getNearestPoints(sample_angles);
}

std::vector<model::Point_t> SphericalSampler::create2BwGrid(
    const std::size_t bw) {
  std::vector<model::Point_t> sample_angles;
  const std::size_t grid = 2*bw - 1; 
  for (std::size_t i = 0; i < grid; ++i){
    for (std::size_t j = 0; j < grid; ++j) {
      model::Point_t p;
      p.x = (M_PI*(2*i+1))/(4*bw);
      p.y = (2*M_PI*j)/(2*bw);
      sample_angles.emplace_back(std::move(p));
    }
  }

  return sample_angles;
}

}
