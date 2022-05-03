#include "phaser/backend/fusion/laplace-pyramid.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <glog/logging.h>
#include <omp.h>

namespace phaser_core {

LaplacePyramid::LaplacePyramid(const float div) : divider_(div) {}

PyramidLevel LaplacePyramid::reduce(
    complex_t* coefficients, const PyramidStruct& py_struct,
    const uint8_t lvl) const {
  CHECK_NOTNULL(coefficients);
  // We will reduce the spectrum.
  const uint32_t lower_bound = py_struct.getLowerBoundForLevel(lvl);
  const uint32_t upper_bound = py_struct.getUpperBoundForLevel(lvl);
  const uint32_t n_low_pass = py_struct.getCoefficientsForLevel(lvl);
  const uint32_t n_laplace = lvl > 0
                                 ? py_struct.getCoefficientsForLevel(lvl - 1)
                                 : py_struct.getInitialCoefficientSize();

  VLOG(2) << "Reducing the bw using [" << lower_bound << ", " << upper_bound
          << "] resulting in " << n_low_pass << " samples.";
  std::vector<complex_t> coeff_low_pass(n_low_pass);
  std::vector<complex_t> coeff_laplace(coefficients, coefficients + n_laplace);

  for (uint32_t i = lower_bound; i < upper_bound; ++i) {
    const uint32_t k = i - lower_bound;
    coeff_low_pass[k][0] = coefficients[i][0];
    coeff_low_pass[k][1] = coefficients[i][1];
    coeff_laplace[i][0] = coeff_laplace[i][0] - coeff_low_pass[k][0];
    coeff_laplace[i][1] = coeff_laplace[i][1] - coeff_low_pass[k][1];
  }

  return std::make_pair(std::move(coeff_low_pass), std::move(coeff_laplace));
}

void LaplacePyramid::expand(
    const std::vector<complex_t>& low_pass,
    std::vector<complex_t>* lapl) const {
  CHECK_NOTNULL(lapl);
  const uint32_t n_coeffs = lapl->size();
  const uint32_t lower_bound = std::round(n_coeffs / divider_);
  const uint32_t upper_bound = n_coeffs - lower_bound;

  VLOG(2) << "Expanding the bw using [" << lower_bound << ", " << upper_bound
          << "] resulting in " << n_coeffs << " samples.";

  // #pragma omp parallel for num_threads(4)
  for (uint32_t i = lower_bound; i < upper_bound; ++i) {
    const uint32_t k = i - lower_bound;
    (*lapl)[i][0] = low_pass[k][0];
    (*lapl)[i][1] = low_pass[k][1];
  }
}

std::vector<complex_t> LaplacePyramid::fuseChannels(
    const std::vector<fftw_complex*>& channels, const uint32_t n_coeffs,
    const uint8_t n_levels) {
  const uint32_t n_channels = channels.size();
  std::vector<std::vector<complex_t>> fused_levels(n_levels);
  std::vector<std::vector<PyramidLevel>> pyramids_per_channel(n_levels);
  PyramidStruct py_struct(n_coeffs, n_levels, divider_);

  // Build the pyramid levels per channel.
  VLOG(2) << "Constructing " << n_levels << " pyramid levels for " << n_channels
          << " channels.";
  std::vector<complex_t*> coefficients(n_channels);
  for (uint32_t i = 0; i < n_channels; ++i) {
    coefficients[i] = reinterpret_cast<complex_t*>(channels[i]);
  }

  for (uint32_t i = 0u; i < n_levels; ++i) {
    std::vector<PyramidLevel> pyramid_level(n_channels);
#pragma omp parallel for num_threads(n_channels) \
    shared(pyramid_level, coefficients)
    for (uint32_t j = 0u; j < n_channels; ++j) {
      VLOG(2) << "Level " << i << " and channel " << j;
      pyramid_level[j] = reduce(coefficients[j], py_struct, i);
      coefficients[j] = pyramid_level[j].first.data();
    }
    fused_levels[i] = fuseLevelByMaxCoeff(pyramid_level, py_struct, i);
    pyramids_per_channel[i] = std::move(pyramid_level);
  }

  // Average the last low pass layer.
  VLOG(2) << "Filtering the low pass layer.";
  std::vector<complex_t> low_pass =
      fuseLastLowPassLayer(pyramids_per_channel.back());

  // Based on the fused levels, reconstruct the signal.
  VLOG(2) << "Reconstructing the fused signal.";
  std::vector<complex_t>* recon = &low_pass;
  for (int8_t i = n_levels - 1; i >= 0; --i) {
    CHECK_NOTNULL(recon);
    expand(*recon, &fused_levels[i]);
    recon = &fused_levels[i];
  }
  CHECK_NOTNULL(recon);
  return *recon;
}

std::vector<complex_t> LaplacePyramid::fuseLevelByMaxCoeff(
    const std::vector<PyramidLevel>& levels_per_channel,
    const PyramidStruct& py_struct, const uint8_t lvl) const {
  CHECK_GT(levels_per_channel.size(), 0);
  const uint32_t n_laplace = lvl > 0
                                 ? py_struct.getCoefficientsForLevel(lvl - 1)
                                 : py_struct.getInitialCoefficientSize();

  std::vector<complex_t> fused(n_laplace);
#pragma omp parallel for num_threads(4) shared(fused)
  for (uint32_t i = 0u; i < n_laplace; ++i) {
    const uint32_t max_channel = findMaxCoeffForChannels(levels_per_channel, i);
    const complex_t& max_coeff = levels_per_channel[max_channel].second[i];
    fused[i][0] = max_coeff[0];
    fused[i][1] = max_coeff[1];
  }
  return fused;
}

std::vector<complex_t> LaplacePyramid::fuseLastLowPassLayer(
    const std::vector<PyramidLevel>& levels_per_channel) const {
  CHECK_GT(levels_per_channel.size(), 0);
  const uint32_t n_coeffs = levels_per_channel[0].first.size();
  std::vector<complex_t> fused(n_coeffs);
#pragma omp parallel for num_threads(4) shared(fused)
  for (uint32_t i = 0; i < n_coeffs; ++i) {
    std::array<double, 2> avg = averageCoeffForChannels(levels_per_channel, i);
    fused[i][0] = avg[0];
    fused[i][1] = avg[1];
  }
  return fused;
}

uint32_t LaplacePyramid::findMaxCoeffForChannels(
    const std::vector<PyramidLevel>& levels_per_channel,
    const uint32_t j) const {
  const uint32_t n_channels = levels_per_channel.size();
  std::vector<double> energies(n_channels);
  for (uint32_t i = 0u; i < n_channels; ++i) {
    const PyramidLevel& level = levels_per_channel[i];
    energies[i] = computeSignalEnergyForLevel(level, j);
  }
  auto it = std::max_element(energies.begin(), energies.end());
  CHECK(it != energies.end());
  return std::distance(energies.begin(), it);
}

double LaplacePyramid::computeSignalEnergyForLevel(
    const PyramidLevel& level, const uint32_t idx) const {
  const std::array<double, 2>& signal = level.second[idx];
  const double energy =
      std::sqrt(signal[0] * signal[0] + signal[1] * signal[1]);
  return energy * energy;
}

std::array<double, 2> LaplacePyramid::averageCoeffForChannels(
    const std::vector<PyramidLevel>& levels_per_channel,
    const uint32_t idx) const {
  const uint32_t n_channels = levels_per_channel.size();
  std::vector<complex_t> signals(n_channels);
  for (uint32_t i = 0; i < n_channels; ++i) {
    const std::vector<complex_t>& low_pass = levels_per_channel[i].first;
    signals[i][0] = low_pass[idx][0];
    signals[i][1] = low_pass[idx][1];
  }
  return averageSignal(signals);
}

std::array<double, 2> LaplacePyramid::averageSignal(
    const std::vector<complex_t>& signals) const {
  double accumulated_real = 0;
  double accumulated_imag = 0;
  double n_signals = static_cast<double>(signals.size());
  for (const complex_t& signal : signals) {
    accumulated_real += signal[0];
    accumulated_imag += signal[1];
  }
  return {accumulated_real / n_signals, accumulated_imag / n_signals};
}

}  // namespace phaser_core
