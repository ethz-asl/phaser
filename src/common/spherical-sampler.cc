#include <packlo/common/spherical-sampler.h>
#include <cmath>

#include <glog/logging.h>

namespace common {

SphericalSampler::SphericalSampler(const int bandwith) 
    : is_initialized_(false) {
  initialize(bandwith);
}

void SphericalSampler::initialize(const int bandwith) {
  const std::vector<common::Point_t> sample_angles = create2BwGrid(bandwith);
  cartesian_grid_ = convertCartesian(sample_angles);
  is_initialized_ = true;
  bandwith_ = bandwith;
}

 void SphericalSampler::sampleUniformly(
    const model::PointCloud &cloud, 
    std::vector<model::FunctionValue>* grid) {
  CHECK(is_initialized_);
  grid->clear();
  cloud.getNearestPoints(cartesian_grid_, grid);
}

std::vector<common::Point_t> SphericalSampler::create2BwGrid(
    const std::size_t bw) {
  std::vector<common::Point_t> sample_angles;
  const std::size_t grid = 2*bw - 1; 
  for (std::size_t i = 0u; i <= grid; ++i){
    const float x = (M_PI*(2*i+1))/(4*bw);
    for (std::size_t j = 0u; j <= grid; ++j) {
      common::Point_t p;
      p.x = x;
      p.y = (2*M_PI*j)/(2*bw);
      sample_angles.emplace_back(std::move(p));
    }
  }

  return sample_angles;
}

std::vector<common::Point_t> SphericalSampler::convertCartesian(
      const std::vector<common::Point_t>& grid) {
  std::vector<common::Point_t> res;
  const float n_grid = static_cast<float>(grid.size()) / 25;
  const float step_distance = 0.00;
  VLOG(1) << "step distance = " << step_distance << " n: " << n_grid;
  float dist = 25.0f;
  for (const common::Point_t& p : grid) {
    common::Point_t cart_p;
    //VLOG(1) << "sin: " << std::sin(p.y) << " cos: " << std::cos(p.y);
    cart_p.x = dist*std::sin(p.x) * std::cos(p.y);
    cart_p.y = dist*std::sin(p.x) * std::sin(p.y);
    cart_p.z = dist*std::cos(p.x);
    res.emplace_back(std::move(cart_p));
    dist += step_distance;
  }
  return res;
}

int SphericalSampler::getInitializedBandwith() const noexcept {
  CHECK(is_initialized_);
  return bandwith_;
}

}
