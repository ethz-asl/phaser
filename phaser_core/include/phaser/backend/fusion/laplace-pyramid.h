#ifndef PHASER_BACKEND_FUSION_LAPLACE_PYRAMID_H_
#define PHASER_BACKEND_FUSION_LAPLACE_PYRAMID_H_

#include <array>
#include <cstdint>
#include <fftw3/fftw3.h>
#include <string>
#include <utility>
#include <vector>

#include "phaser/backend/fusion/pyramid-struct.h"

namespace phaser_core {

// using complex_t = std::complex<double>;
// using complex_t = double[2];
using complex_t = std::array<double, 2>;

using PyramidLevel = std::pair<std::vector<complex_t>, std::vector<complex_t>>;

class LaplacePyramid {
 public:
  explicit LaplacePyramid(const float div = 4.0);
  PyramidLevel reduce(
      complex_t* coefficients, const PyramidStruct& py_struct,
      const uint8_t lvl) const;
  void expand(
      const std::vector<complex_t>& low_pass,
      std::vector<complex_t>* lapl) const;
  std::vector<complex_t> fuseChannels(
      const std::vector<fftw_complex*>& channels, const uint32_t n_coeffs,
      const uint8_t n_levels);
  std::vector<complex_t> fuseLevelByMaxCoeff(
      const std::vector<PyramidLevel>& level, const PyramidStruct& py_struct,
      const uint8_t lvl) const;
  std::vector<complex_t> fuseLastLowPassLayer(
      const std::vector<PyramidLevel>& levels_per_channel) const;

 private:
  uint32_t findMaxCoeffForChannels(
      const std::vector<PyramidLevel>& levels_per_channel,
      const uint32_t idx) const;
  double computeSignalEnergyForLevel(
      const PyramidLevel& level, const uint32_t idx) const;

  std::array<double, 2> averageCoeffForChannels(
      const std::vector<PyramidLevel>& levels_per_channel,
      const uint32_t idx) const;
  std::array<double, 2> averageSignal(
      const std::vector<complex_t>& signals) const;
  void writeToFile(
      const std::string& filename, const std::vector<complex_t>& signal);

  const float divider_;
};

}  // namespace phaser_core

#endif  // PHASER_BACKEND_FUSION_LAPLACE_PYRAMID_H_
