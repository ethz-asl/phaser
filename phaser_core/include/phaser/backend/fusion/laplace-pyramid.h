#ifndef PHASER_BACKEND_FUSION_LAPLACE_PYRAMID_H_
#define PHASER_BACKEND_FUSION_LAPLACE_PYRAMID_H_

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

#include <fftw3/fftw3.h>

namespace fusion {

// using complex_t = std::complex<double>;
using complex_t = double[2];

using PyramidLevel = std::pair<std::vector<complex_t>, std::vector<complex_t>>;

class LaplacePyramid {
 public:
  explicit LaplacePyramid(const float div = 4.0);
  PyramidLevel reduce(fftw_complex* coefficients, const uint32_t n_coeffs);
  void expand(
      const std::vector<complex_t>& low_pass, std::vector<complex_t>* lapl);
  void fuseChannels(
      const std::vector<fftw_complex*>& channels, const uint32_t n_coeffs,
      const uint32_t n_levels);
  std::vector<complex_t> fuseLevelByMaxCoeff(
      const std::vector<PyramidLevel>& level, const uint32_t n_coeffs);
  std::vector<complex_t> fuseLastLowPassLayer(
      const std::vector<PyramidLevel>& levels_per_channel);

 private:
  uint32_t findMaxCoeffForChannels(
      const std::vector<PyramidLevel>& levels_per_channel, const uint32_t idx);
  double computeSignalEnergyForLevel(
      const PyramidLevel& level, const uint32_t idx);

  std::array<double, 2> averageCoeffForChannels(
      const std::vector<PyramidLevel>& levels_per_channel, const uint32_t idx);
  std::array<double, 2> averageSignal(const std::vector<complex_t>& signals);

  const float divider_;
};

}  // namespace fusion

#endif  // PHASER_BACKEND_FUSION_LAPLACE_PYRAMID_H_
