#ifndef PHASER_BACKEND_CORRELATION_SPHERICAL_COMBINED_WORKER_H_
#define PHASER_BACKEND_CORRELATION_SPHERICAL_COMBINED_WORKER_H_

#include <memory>
#include <vector>

#include "phaser/backend/correlation/spherical-correlation.h"
#include "phaser/common/base-worker.h"
#include "phaser/common/spherical-sampler.h"
#include "phaser/common/thread-pool.h"
#include "phaser/model/point-cloud.h"

namespace phaser_core {

class SphericalCombinedWorker : public common::BaseWorker {
 public:
  explicit SphericalCombinedWorker(
      const model::FunctionValueVec& f_values,
      const model::FunctionValueVec& h_values);

  void run() override;

  std::vector<double> getCorrelation() const noexcept;
  const SphericalCorrelation& getCorrelationObject() const noexcept;
  void shutdown();

 private:
  SphericalCorrelationPtr sph_corr_;
  const model::FunctionValueVec& f_values_;
  const model::FunctionValueVec& h_values_;
};
using SphericalCombinedWorkerPtr = std::shared_ptr<SphericalCombinedWorker>;

}  // namespace phaser_core

#endif  // PHASER_BACKEND_CORRELATION_SPHERICAL_COMBINED_WORKER_H_
