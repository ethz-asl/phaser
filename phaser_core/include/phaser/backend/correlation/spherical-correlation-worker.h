#ifndef PHASER_BACKEND_CORRELATION_SPHERICAL_CORRELATION_WORKER_H_
#define PHASER_BACKEND_CORRELATION_SPHERICAL_CORRELATION_WORKER_H_

#include "phaser/common/base-worker.h"
#include "phaser/model/point-cloud.h"

namespace correlation {

class SphericalCorrelationWorker : public BaseWorker {
 public:
  explicit SphericalCorrelationWorker(
      const model::PointCloud& source, const model::PointCloud& target);

  void run() override;

  void isFinished() const override;

 private:
};

}  // namespace correlation

#endif  // PHASER_BACKEND_CORRELATION_SPHERICAL_CORRELATION_WORKER_H_
