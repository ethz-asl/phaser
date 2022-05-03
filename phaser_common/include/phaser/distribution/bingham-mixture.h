#ifndef PHASER_DISTRIBUTION_BINGHAM_MIXTURE_H_
#define PHASER_DISTRIBUTION_BINGHAM_MIXTURE_H_

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "phaser/distribution/base-distribution.h"
#include "phaser/distribution/bingham.h"

namespace common {

class BinghamMixture : public BaseDistribution {
 public:
  explicit BinghamMixture(
      const std::vector<Bingham>& binghams, const Eigen::VectorXd& weights);

  Eigen::VectorXd getEstimate() const override;

  uint16_t getSampleSize() const noexcept;
  uint16_t& getSampleSize();

  const Eigen::VectorXd& getMixtureMode() const;
  const Eigen::MatrixXd& getMixtureS2() const;
  const Eigen::VectorXd& getMixtureZ() const;
  const Eigen::MatrixXd& getMixtureM() const;

 private:
  void calcMixtureParametersGlover();

  const std::vector<Bingham>& binghams_;
  const Eigen::VectorXd weights_;

  Eigen::VectorXd mode_;
  Eigen::MatrixXd S2_;
  Eigen::VectorXd Z_;
  Eigen::MatrixXd M_;
  uint8_t dim_;
  uint16_t bingham_sample_size_;
};

using BinghamMixturePtr = std::shared_ptr<BinghamMixture>;

}  // namespace common

#endif  // PHASER_DISTRIBUTION_BINGHAM_MIXTURE_H_
