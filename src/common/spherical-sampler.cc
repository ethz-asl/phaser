#include <packlo/common/spherical-sampler.h>
#include <cmath>

#include <glog/logging.h>

namespace common {

std::vector<float> SphericalSampler::sampleUniformly(
		const model::PointCloud &cloud,
    const std::size_t bw) {
  std::vector<model::Point_t> sample_angles = create2BwGrid(bw, true);
  std::vector<model::Point_t> converted = convertCartesian(sample_angles);

  return cloud.getNearestPoints(converted);
}

std::vector<model::Point_t> SphericalSampler::create2BwGrid(
    const std::size_t bw, const bool cartesian) {
  std::vector<model::Point_t> sample_angles;
  const std::size_t grid = 2*bw - 1; 
  for (std::size_t i = 0; i <= grid; ++i){
    for (std::size_t j = 0; j <= grid; ++j) {
      model::Point_t p;
      p.x = (M_PI*(2*i+1))/(4*bw);
      p.y = (2*M_PI*j)/(2*bw);
      sample_angles.emplace_back(std::move(p));
    }
  }

  return sample_angles;
}

std::vector<model::Point_t> SphericalSampler::convertCartesian(
      std::vector<model::Point_t>& grid) {
  std::vector<model::Point_t> res;
  for (const model::Point_t& p : grid) {
    model::Point_t cart_p;
    cart_p.x = 10*std::sin(p.x) * std::cos(p.y);
    cart_p.y = 10*std::sin(p.x) * std::sin(p.y);
    cart_p.z = 10*std::cos(p.x);
    res.emplace_back(std::move(cart_p));
  }
  return res;
}

}
