#ifndef PHASER_BACKEND_CORRELATION_SPHERICAL_CORRELATION_WORKER_H_
#define PHASER_BACKEND_CORRELATION_SPHERICAL_CORRELATION_WORKER_H_

#include <memory>
#include <vector>

#include "phaser/backend/correlation/spherical-correlation.h"
#include "phaser/common/base-worker.h"
#include "phaser/common/spherical-sampler.h"
#include "phaser/model/point-cloud.h"

namespace correlation {

class SphericalCorrelationWorker : public common::BaseWorker {
 public:
  explicit SphericalCorrelationWorker(
      model::PointCloudPtr target, model::PointCloudPtr source,
      const uint16_t bandwidth);

  void run() override;

  std::vector<double> getCorrelation() const noexcept;
  const SphericalCorrelation& getCorrelationObject() const noexcept;

 private:
  model::PointCloudPtr source_;
  model::PointCloudPtr target_;
  common::SphericalSampler sampler_;
  SphericalCorrelation sph_corr_;
};
using SphericalCorrelationWorkerPtr =
    std::shared_ptr<SphericalCorrelationWorker>;

}  // namespace correlation

#endif  // PHASER_BACKEND_CORRELATION_SPHERICAL_CORRELATION_WORKER_H_
