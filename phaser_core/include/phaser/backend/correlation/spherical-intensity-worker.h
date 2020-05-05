#ifndef PHASER_BACKEND_CORRELATION_SPHERICAL_INTENSITY_WORKER_H_
#define PHASER_BACKEND_CORRELATION_SPHERICAL_INTENSITY_WORKER_H_

#include <memory>
#include <vector>

#include "phaser/backend/correlation/spherical-correlation.h"
#include "phaser/common/base-worker.h"
#include "phaser/common/spherical-sampler.h"
#include "phaser/common/thread-pool.h"
#include "phaser/model/point-cloud.h"

namespace correlation {

class SphericalIntensityWorker : public common::BaseWorker {
 public:
  explicit SphericalIntensityWorker(
      const model::FunctionValueVec& f_values,
      const model::FunctionValueVec& h_values, const uint16_t bandwidth);

  void run() override;

  std::vector<double> getCorrelation() const noexcept;
  const SphericalCorrelation& getCorrelationObject() const noexcept;

 private:
  SphericalCorrelation sph_corr_;
  const model::FunctionValueVec& f_values_;
  const model::FunctionValueVec& h_values_;
  const uint32_t bw_;
};
using SphericalIntensityWorkerPtr = std::shared_ptr<SphericalIntensityWorker>;

}  // namespace correlation

#endif  // PHASER_BACKEND_CORRELATION_SPHERICAL_INTENSITY_WORKER_H_
