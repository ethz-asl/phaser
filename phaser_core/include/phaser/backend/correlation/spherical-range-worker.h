#ifndef PHASER_BACKEND_CORRELATION_SPHERICAL_RANGE_WORKER_H_
#define PHASER_BACKEND_CORRELATION_SPHERICAL_RANGE_WORKER_H_

#include <memory>
#include <vector>

#include "phaser/backend/correlation/spherical-correlation-low-pass.h"
#include "phaser/backend/correlation/spherical-correlation.h"
#include "phaser/common/base-worker.h"
#include "phaser/common/spherical-sampler.h"
#include "phaser/common/thread-pool.h"
#include "phaser/model/point-cloud.h"

namespace phaser_core {

class SphericalRangeWorker : public common::BaseWorker {
 public:
  explicit SphericalRangeWorker(
      const model::FunctionValueVec& f_values,
      const model::FunctionValueVec& h_values);

  void run() override;

  std::vector<double> getCorrelation() const noexcept;
  const SphericalCorrelation& getCorrelationObject() const noexcept;

 private:
  SphericalCorrelationLowPass sph_corr_;
  const model::FunctionValueVec& f_values_;
  const model::FunctionValueVec& h_values_;
};

using SphericalRangeWorkerPtr = std::shared_ptr<SphericalRangeWorker>;

}  // namespace phaser_core

#endif  // PHASER_BACKEND_CORRELATION_SPHERICAL_RANGE_WORKER_H_
