#include "phaser/backend/fusion/laplace-pyramid.h"

#include <algorithm>
#include <cmath>

#include <glog/logging.h>

namespace fusion {

LaplacePyramid::LaplacePyramid(const float div) : divider_(div) {}

PyramidLevel LaplacePyramid::reduce(
    fftw_complex* coefficients, const uint32_t n_coeffs) {
  CHECK_NOTNULL(coefficients);
  // We will reduce the spectrum by half the number of coefficients.
  const uint32_t lower_bound = std::round(n_coeffs / divider_);
  const uint32_t upper_bound = n_coeffs - lower_bound;
  const uint32_t n_low_pass = upper_bound - lower_bound;
  std::vector<complex_t> coeff_low_pass_full(n_coeffs);
  std::vector<complex_t> coeff_low_pass(n_low_pass);
  std::vector<complex_t> coeff_laplace(n_coeffs);

  VLOG(1) << "[LaplacePyramid] lower: " << lower_bound
          << ", upper: " << upper_bound << " n_low_pass: " << n_low_pass;

  uint32_t k = 0;
  for (uint32_t i = lower_bound; i < upper_bound; ++i) {
    coeff_low_pass_full[i][0] = coefficients[i][0];
    coeff_low_pass_full[i][1] = coefficients[i][1];
    coeff_low_pass[k][0] = coefficients[i][0];
    coeff_low_pass[k][1] = coefficients[i][1];
    ++k;
  }

  for (uint32_t i = 0; i < n_coeffs; ++i) {
    coeff_laplace[i][0] = coefficients[i][0] - coeff_low_pass_full[i][0];
    coeff_laplace[i][1] = coefficients[i][1] - coeff_low_pass_full[i][1];
  }
  return std::make_pair(std::move(coeff_low_pass), std::move(coeff_laplace));
}

void LaplacePyramid::expand(
    const std::vector<complex_t>& low_pass, std::vector<complex_t>* lapl) {
  CHECK_NOTNULL(lapl);
  const uint32_t n_coeffs = lapl->size();
  const uint32_t lower_bound = std::round(n_coeffs / divider_);
  const uint32_t upper_bound = n_coeffs - lower_bound;

  uint32_t k = 0;
  for (uint32_t i = lower_bound; i < upper_bound; ++i) {
    (*lapl)[i][0] = low_pass[k][0];
    (*lapl)[i][1] = low_pass[k][1];
    ++k;
  }
}

void LaplacePyramid::fuseChannels(
    const std::vector<fftw_complex*>& channels, const uint32_t n_coeffs,
    const uint32_t n_levels) {
  const uint32_t n_channels = channels.size();
  for (uint32_t i = 0u; i < n_levels; ++i) {
    std::vector<PyramidLevel> pyramid_level(n_channels);
    for (uint32_t j = 0u; j < n_channels; ++j) {
      pyramid_level[j] = reduce(channels[j], n_coeffs);
    }
  }
}

void LaplacePyramid::fuseLevelByMaxCoeff(
    const std::vector<PyramidLevel>& levels_per_channel) {
  CHECK_GT(levels_per_channel.size(), 0);

  const uint32_t n_coeffs = levels_per_channel[0].first.size();
  std::vector<complex_t> fused(n_coeffs);
  for (uint32_t i = 0u; i < n_coeffs; ++i) {
    const uint32_t max_channel = findMaxCoeffForChannels(levels_per_channel, i);
    const complex_t& max_coeff = levels_per_channel[max_channel].second[i];
    fused[i][0] = max_coeff[0];
    fused[i][1] = max_coeff[1];
  }
}

uint32_t LaplacePyramid::findMaxCoeffForChannels(
    const std::vector<PyramidLevel>& levels_per_channel, const uint32_t j) {
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
    const PyramidLevel& level, const uint32_t idx) {
  const double* signal = level.second[idx];
  const double energy =
      std::sqrt(signal[0] * signal[0] + signal[1] * signal[1]);
  return energy * energy;
}

}  // namespace fusion
