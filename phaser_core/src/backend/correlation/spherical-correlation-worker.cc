#include "phaser/backend/correlation/spherical-correlation-worker.h"

#include <glog/logging.h>

namespace correlation {

SphericalCorrelationWorker::SphericalCorrelationWorker(
    model::PointCloudPtr target, model::PointCloudPtr source,
    const uint16_t bandwidth)
    : source_(source), target_(target), sampler_(bandwidth) {}

void SphericalCorrelationWorker::run() {
  VLOG(1) << "[SphRegistrationWorker] Estimating rotation...";
  source_->initialize_kd_tree();
  target_->initialize_kd_tree();

  // Sample the sphere at the grid points.
  std::vector<model::FunctionValue> f_values;
  std::vector<model::FunctionValue> h_values;
  sampler_.sampleUniformly(*target_, &f_values);
  sampler_.sampleUniformly(*source_, &h_values);

  sph_corr_.correlateSignals(
      f_values, h_values, sampler_.getInitializedBandwith());

  is_completed_ = true;
}

std::vector<double> SphericalCorrelationWorker::getCorrelation() const
    noexcept {
  return sph_corr_.getCorrelation();
}

const SphericalCorrelation& SphericalCorrelationWorker::getCorrelationObject()
    const noexcept {
  return sph_corr_;
}

}  // namespace correlation
